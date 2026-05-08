# EGOBatteryCommunication

Arduino Mega sketch that decodes the 1-wire serial protocol on an EGO power-tool
battery's `D` pin. Captures live tool↔battery exchanges, prints decoded frames
over USB serial, and can also act as a minimal tool by transmitting commands.

## Hardware wiring

The exact wiring depends on whether you want to **transmit commands** as
well as listen, or just **passively sniff** an existing tool↔battery
conversation. Pick one — they're not the same circuit.

### Active wiring (TX + RX)

To both listen and inject commands, the bus needs a HIGH-side path to 5 V so
the line idles HIGH between bits. Two options, both work:

```
                  +-- D pin
                  |
5V --[ R or D ]--+-+-- Pin 2  (digital — edge detection / TX)
                  |
                  +-- A0     (analog — direction detection)
GND -------------- - pin
```

| `R or D`                         | Idle HIGH | Notes                                                                                                                                                                                                       |
|----------------------------------|-----------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| **470 Ω resistor**               | ~5 V open, drops with load | Simple, what was used in the original Arduino sketch. The actual idle HIGH depends on whatever else is loading the bus.                                                                              |
| **1N4148 diode + 100 Ω series**  | ~4.3 V    | **Recommended.** The diode's 0.7 V forward drop pins the idle HIGH right at the ~4 V level a real EGO tool drives, so the bus looks "natural" to the battery. The diode also blocks reverse current back into the Mega's 5 V rail. The 100 Ω caps the current when Pin 2 pulls LOW. |

Wire the diode anode-to-5V, cathode toward the bus (current flows 5V → bus
only). Pin 2 stays `INPUT` at idle and flips to `OUTPUT/LOW` only during TX.
**Never drive `OUTPUT HIGH`** — the diode (or resistor) is what creates the
HIGH level.

A0 taps the same node and is sampled at the start of each frame to determine
who's driving the line (see [Direction detection](#direction-detection)).

### Passive-sniffing wiring (RX only)

For sniffing alone — capturing real tool↔battery exchanges without ever
transmitting — **remove the diode/resistor and don't feed 5 V at all**. The
real tool already drives the bus HIGH; adding our own pull-up shifts the
idle level and risks being noticed by the BMS.

```
                +-- D pin   (driven by the real tool/battery)
                +-- Pin 2   (INPUT, high impedance)
                +-- A0      (INPUT, direction detection)
GND ----------- - pin
```

A 10 kΩ in series between Pin 2 and the bus is optional but a sensible
belt-and-suspenders: if Pin 2 ever gets configured as `OUTPUT` by mistake,
the resistor prevents it from fighting the tool's drive. Auto-ADJUST is OFF
by default at boot, so the firmware won't TX unless you trigger it via the
serial keys.

## Wire format

- Short LOW (~200 µs) = bit `1`
- Long LOW  (~600 µs) = bit `0`
- Sync = LOW ~1250 µs at start of a frame group (when present)
- Bytes are LSB-first; commands are byte-reversed on the wire (e.g.
  `RD_VOL` → `LOV_DR`)
- CRC = Dallas 1-Wire CRC8 (poly `0x31`, refin/refout, init 0)
- 9-byte command frames carry a leading framing bit (73 bits total) that is
  dropped before byte extraction
- A HIGH gap longer than `FRAME_GAP_US` (1500 µs) marks end-of-frame.
  Inter-bit HIGH is 200–600 µs, so 1500 µs cleanly separates back-to-back
  tool query / battery response without false splits inside a frame.

## Direction detection

Tool and battery drive the bus to different LOW levels:

| Driver  | Bus during LOW | ADC reading (5 V Vref, 10-bit) |
|---------|----------------|--------------------------------|
| tool    | ~0 V           | ≈ 0                            |
| battery | ~0.7–0.8 V     | ≈ 140–165                      |

The sketch samples A0 once at the start of every frame (in the falling-edge
ISR, with the ADC prescaler set to 16 for ~13 µs conversions) and tags each
RX line `TOOL-> ` or `BATT-> ` based on a threshold of 82 (≈ 0.4 V). This is
ground truth and replaces the older byte[0]==0 heuristic that mislabeled
`START_` and `ADJUST`.

When the ADC reading isn't available, the decoder falls back to the byte[0]
heuristic for read commands, and prints `?--->` in the direction column.

## Frame types observed

| Bits | Bytes | Use                                                                |
|-----:|------:|--------------------------------------------------------------------|
|   40 |     5 | ID frame from battery (see [ID frame structure](#id-frame-structure)) |
|   73 |     9 | Command frame (battery- or tool-originated; see Protocol flow)       |
|   96 |    12 | Tool-originated handshake blob seen with the diagnostic-grade smart tool on Gen2. **Tool-class extension, not part of the base protocol** — the EGO charger talking to the same Gen2 battery skips it entirely. Currently CRC-fails under our 9-byte parser. |
|  178 |  22+2b| Tool-originated handshake blob seen with the diagnostic-grade smart tool on Gen1. Same role as the 96b blob; tail contains ASCII `BA3`, likely a model-string fragment. Also a tool-class extension (chargers may not send this either — needs confirmation). |

When `FRAME_GAP_US` was 5000 µs, two 73-bit frames sometimes glued into a
single 146-bit blob. The current 1500 µs threshold splits them.

### ID frame structure

The 5-byte ID frame is the battery's heartbeat. Layout (in capture order):

| Offset | Value (10 Ah) | Value (5 Ah) | Notes                                        |
|-------:|---------------|--------------|----------------------------------------------|
|      0 | `AA`          | `AA`         | constant — sync / header marker              |
|      1 | `0B`          | `19`         | varies per battery (model / capacity code?)  |
|      2 | `4A` (`'J'`)  | `50` (`'P'`) | varies per battery — looks like an ASCII letter |
|      3 | `49` (`'I'`)  | `49` (`'I'`) | constant — likely "ID" tag                   |
|      4 | `D5`          | `DB`         | Dallas CRC8 over bytes 0..3                  |

Bytes 1–2 differentiate batteries; bytes 0 and 3 are constant across the two
samples we've captured. More captures (different generations / capacities)
needed to fully decode the bytes 1–2 encoding.

## Protocol flow

The battery is the conversation initiator. The exact flow depends on the
battery generation.

### Gen2 battery + smart source (charger or smart tool)

```
battery →  ID                              \
battery →  ID                               >  heartbeat + session invite, ~580 ms
battery →  START_  data=0x0000             /
tool    →  [96-bit handshake blob]            (smart-tool extension — chargers skip this)
tool    →  ADJUST  data=0x0000                (always 0x0000 from the tool side)
battery →  ADJUST  data=0x00A0                (Gen2 reply — `0x00A0` is the battery's marker)
tool    →  RD_SPC, RD_CAP, RD_FSH             (read once)
tool    →  RD_TMP × 2  →  RD_VOL × 14  →  repeat (~3 s/cycle, no re-handshake)
battery →  (each response)
```

### Gen1 battery + smart tool

```
battery →  ID                              \
battery →  ID                                |  first connection only:
tool    →  [178-bit handshake blob]          |  no START_, tool initiates
battery →  ADJUST  data=0x0000               /

(then per cycle, every ~5 s:)
battery →  ID
battery →  ID
battery →  START_  data=0x0000
tool    →  ADJUST  data=0x0000               (always 0x0000 from the tool side)
battery →  ADJUST  data=0x0000               (Gen1 reply: battery echoes 0x0000)
tool    →  RD_SPC, RD_CAP, RD_FSH            (re-read every cycle)
tool    →  RD_TMP × 2  →  RD_VOL × 14
[~1 s sustained LOW on the bus, battery LED flashes through several colors]
... next cycle
```

### ADJUST exchange — Gen1 vs Gen2

Tools always send `ADJUST data=0x0000`. The **battery's reply** is what
varies by generation, and that's how we tell them apart:

| Direction      | Gen1     | Gen2     |
|----------------|----------|----------|
| tool → batt    | `0x0000` | `0x0000` |
| batt → tool    | `0x0000` (echo) | `0x00A0` (Gen2 marker) |

This corrects an earlier interpretation: the lone `0x00A0` ADJUST seen in
the original capture (before direction detection existed) was the battery's
reply, not a smart-tool transmission.

### Notable Gen1-specific behavior

- The 178-bit handshake blob contains an ASCII fragment `BA3` near the tail,
  hinting it carries a tool/battery model identifier.
- The tool re-reads SPC/CAP/FSH every cycle instead of caching them after
  the first session, suggesting Gen1 BMS doesn't persist a "session is
  established" state. (Gen2 caches.)
- **Sustained ~1 s LOW between cycles** during which the battery's LED
  flashes through several colors. This looks like a battery-side state
  display window between protocol exchanges (capacity gauge / status
  feedback). The Arduino logs it as `(long LOW ~1002000us)`.

### Charger + Gen2 battery

The charger uses an entirely different command vocabulary from the
diagnostic protocol — it doesn't read individual cells, it lets the BMS
drive the current setpoint and just reports back what it's delivering.

```
battery →  ID                              \
battery →  ID                                >  heartbeat + invite, ~580 ms
battery →  START_  data=0x0000              /
tool    →  START_  data=<charger id>          (non-zero, varies per charger model)
battery →  START_  data=0x0000                (re-issued)
tool    →  START_  data=<charger id>          (re-issued)
battery →  EVACHG  data=0x0403                (enter charge mode)
tool    →  EVACHG  data=0x0403                (ack)
battery →  GETCG1  data=0x0000   ; tool  →  OUTCG1  data=0x0000
battery →  GET_VM  data=0x0000   ; tool  →  OUT_VM  data=0x1770   (60.00 V out)
battery →  GET_CM  data=0x0000   ; tool  →  OUT_CM  data=0x012C   (3.00 A out)
battery →  FAN_ON  data=0x0064   ; tool  →  FAN_ON  data=0x0005   (battery wants 100, charger ramps from 5)
battery →  EN_OUT  data=0x0000   ; tool  →  EN_OUT  data=0x0001   (turn output on)

(then steady-state telemetry loop, ~200 ms per exchange:)
  ADDCUR/OUTCUR × 4   →   CHGPCT   →   FAN_ON
  ADDCUR/OUTCUR × 4   →   CHGPCT   →   FAN_ON
  ...
```

Notable charging-mode behavior:

- **Closed-loop CC charging.** Battery repeatedly sends `ADDCUR data=5`
  (asking for +5 units of current); charger echoes `OUTCUR` with the actual
  delivered current, which ramps monotonically. When the battery drops to
  `ADDCUR=2`, OUTCUR's rate of climb slows — the BMS is fine-tuning. As
  the battery approaches full, the BMS switches from `ADDCUR` to `DECCUR`
  and `OUTCUR` ramps **down** for the taper / balance phase. The charger
  doesn't decide how much to push; the battery does.
- **`CHGFUL` fires before the taper actually finishes.** Observed roughly
  the moment the BMS is happy with the battery's state of charge for use,
  followed by ~70 s more `OUTCUR` taper driven by `DECCUR`. The
  charger's "all green" LED indicator and CHGFUL appear to coincide; the
  fan keeps running until the actual current settles.
- **`CHGPCT` can exceed 100%.** Raw SOC values up to 117 % observed during
  absorption. The BMS appears to use a percentage that overshoots before
  settling. Display layers should cap to 100 % for sanity.
- **Charger never reads cells / temps.** No `RD_VOL`, `RD_TMP`, `RD_SPC`,
  `RD_CAP`, or `RD_FSH` in the entire charge session — the charger trusts
  the BMS via `ADDCUR` and `CHGPCT`.
- **`CHGPCT` reports SOC.** Battery sends actual percentage in real time
  (28 → 29 → 30 % observed). Charger always replies `0x0001` (ack/seen).
- **`START_` from the charger varies per charger model.** Observed: `0x00D7`
  (215) on a 3 A charger, `0x000A` (10) on an 8 A charger. Two-sample
  evidence — probably a model / capability code, but not a session counter.

### Charger + Gen2 battery — already full

If the battery is already at full charge when plugged in, the charger
recognises this after the standard handshake and skips straight into a
graceful end-of-session sequence. **No `EN_OUT`, no `ADDCUR`/`OUTCUR`,
no telemetry loop** — the charger never enables its output and both
sides go to sleep within ~3 seconds of plug-in:

```
battery →  ID, ID                              (heartbeat)
battery →  START_  data=0x0000
tool    →  START_  data=<charger id>
battery →  EVACHG  data=0x0000   ; tool  →  EVACHG  data=0x0000
battery →  GETCG1  data=0x0000   ; tool  →  OUTCG1  data=0x1000
battery →  GET_VM  data=0x0000   ; tool  →  OUT_VM  data=0x1770   (60.00 V advertised)
battery →  GET_CM  data=0x0000   ; tool  →  OUT_CM  data=0x0320   (8.00 A advertised — capability)
battery →  FANOFF  data=0x0000   ; tool  →  FANOFF  data=0x0000   (skip the fan-ramp branch)
battery →  DISOUT  data=0x0000   ; tool  →  DISOUT  data=0x0005   (disable output; tool ack with 5)
battery →  CHGFUL  data=0x0000   ; tool  →  CHGFUL  data=0x0000   (full handshake)
battery →  SLEEP_  data=0x0000   ; tool  →  SLEEP_  data=0x0000   (terminate session)
[bus idle]
```

Notes:
- **The fork happens after `OUT_CM`.** With a not-full battery the charger
  sends `FAN_ON` and ramps; with a full battery it sends `FANOFF` and
  jumps to the shutdown branch.
- `EVACHG`/`OUTCG1` payloads also differ from the active-charge case
  (`0x0000` vs `0x0403`, `0x1000` vs `0x0000`). The full-battery payloads
  may be flags/state codes; not enough samples yet to be sure.
- `OUT_CM` here is the charger's **rated** output current, not a setpoint
  for an ongoing charge — the output is never enabled this session.

### "Dumb" tool (any battery)

A dumb tool (drill, blower, …) is silent on the D pin — it just draws
current. The battery sends its `ID, ID, START_` invite ~10 times unanswered,
then drops the START_ retries and keeps only the periodic ID heartbeat
(~180 ms cadence). Power delivery is not gated by the D-pin protocol.

This was confirmed across two batteries (10 Ah and 5 Ah): both produced the
same retry-then-heartbeat pattern, only the ID payload differed. The
direction tag in every captured frame was `BATT-> `, confirming the dumb
tool stays silent throughout.

## Commands

All 9-byte frames have the layout:

```
byte[0..1]   uint16 LE data
byte[2..7]   command name byte-reversed (ASCII)
byte[8]      Dallas CRC8 over byte[0..7]
```

There are two distinct command vocabularies depending on what the battery is
doing: **diagnostic / discharge** (in a tool) and **charging** (in a
charger). Both use the same 9-byte frame structure, but the command names
and the conversation pattern are different.

### Diagnostic / discharge commands

| Command   | Wire bytes (rev) | Direction        | Notes                                                    |
|-----------|------------------|------------------|----------------------------------------------------------|
| `START_`  | `_TRATS`         | **batt → tool**  | Session invite. Data `0x0000`. Sent after the ID×2 heartbeat (and skipped on the first Gen1 connection — see Protocol flow). |
| `ADJUST`  | `TSUJDA`         | **tool → batt** *and* **batt → tool** | Session handshake. Tool always sends `0x0000`. Battery's reply differs by generation: Gen1 echoes `0x0000`, Gen2 replies `0x00A0`. The `0x00A0` reply is the cleanest Gen2 marker we have. |
| `RD_VOL`  | `LOV_DR`         | tool ⇄ batt      | Tool: `byte[1]=cell_idx (0..13)`, `byte[0]=0x00`. Batt: voltage in centivolts (`v/100` V). |
| `RD_TMP`  | `PMT_DR`         | tool ⇄ batt      | Tool: `byte[1]=sensor_idx`. Batt: temperature, units appear to be °F.       |
| `RD_SPC`  | `CPS_DR`         | tool ⇄ batt      | Tool: `0x0000`. Batt: `byte[1]=S-count`, `byte[0]=model`. **`model=1` on Gen1, `model=3` on Gen2** (one-sample evidence — may be a generation marker). |
| `RD_CAP`  | `PAC_DR`         | tool ⇄ batt      | Tool: `0x0000`. Batt: Ah/cell × 100.        |
| `RD_FSH`  | `HSF_DR`         | tool ⇄ batt      | Tool: `Q=0x3800`. Batt: status; `byte[0]=0xFF` = Gen1 stub, otherwise Gen2 status bits. |

### Charging commands

Used between the charger and the battery. Different conversation pattern:
the battery commands the charger (closed-loop CC), the charger reports back
what it's actually doing.

| Command   | Wire bytes (rev) | Direction        | Notes                                                    |
|-----------|------------------|------------------|----------------------------------------------------------|
| `START_`  | `_TRATS`         | **batt → tool** *and* **tool → batt** | Battery sends `0x0000` (same as diagnostic). Charger replies with its own `START_` payload that **varies per charger model** — observed `0x00D7` (215) on a 3 A charger, `0x000A` (10) on an 8 A charger. Likely a model / capability code. |
| `EVACHG`  | `GHCAVE`         | both             | "Enter charge mode" (electric-vehicle-adjust-charge?). `data=0x0403` (1027) on an active charge; `data=0x0000` on a full-battery session. Looks like a mode/state marker. |
| `GETCG1` / `OUTCG1` | `1GCTEG` / `1GCTUO` | batt / tool | "Charge stage 1" query/reply. Active-charge session: both `0x0000`. Full-battery session: charger replies `0x1000` (4096). Possibly a state flag. |
| `GET_VM` / `OUT_VM` | `MV_TEG` / `MV_TUO` | batt / tool | Battery asks the charger's output voltage. `OUT_VM` data is centivolts (e.g. `0x1770 = 6000` → 60.00 V). On a full-battery session this is the charger's rated voltage, not a live setpoint. |
| `GET_CM` / `OUT_CM` | `MC_TEG` / `MC_TUO` | batt / tool | Battery asks the charger's output current. `OUT_CM` data is centiamps (e.g. `0x012C = 300` → 3.00 A; `0x0320 = 800` → 8.00 A). On a full-battery session this is the charger's **rated** capability, not an ongoing setpoint. |
| `EN_OUT`  | `TUO_NE`         | tool → batt      | Charger toggles its output: `data=0x0001` enables, `0x0000` disables. **Skipped on a full-battery session** — see `DISOUT`. |
| `DISOUT`  | `TUOSID`         | both             | Disable output. Sent in the full-battery end-of-session sequence in place of `EN_OUT`. Battery sends `0x0000`, charger acks with `0x0005` (5; meaning unclear — possibly seconds). |
| `CHGING`  | `GNIGHC`         | both             | "Charging" heartbeat / handshake. Data `0x0000` both sides. |
| `CHGFUL`  | `LUFGHC`         | both             | "Charge full." BMS signals the battery is **functionally ready to use** — *not* "charging is done". Observed firing **mid-session** during a real charge cycle: ~70 s of `OUTCUR` taper continued after `CHGFUL` (194 → 164 cA), driven by `DECCUR>=0` from the BMS. Also used in the already-full plug-in sequence (where `CHGFUL` fires within ~3 s with no charging in between). Both sides exchange `data=0x0000`. |
| `ADDCUR`  | `RUCDDA`         | batt → tool      | Battery requests an increment to charge current (delta in unknown unit; observed values 2 and 5). Drives the closed-loop ramp **up**. Dominant during early charging; stops being sent once the BMS is into the taper phase. Not seen at all on an already-full session. |
| `DECCUR`  | `RUCCED`         | batt → tool      | Battery requests a *decrement* to charge current — counterpart to `ADDCUR`, dominant during the taper / top-off phase. Observed values 0..3 (data=0 most common). Charger doesn't echo `DECCUR`; it just lowers its `OUTCUR` in response. |
| `OUTCUR`  | `RUCTUO`         | tool → batt      | Charger reports actual current it's delivering. Climbs monotonically while battery sends `ADDCUR>0`; tapers monotonically while battery sends `DECCUR>=0`. |
| `FAN_ON`  | `NO_NAF`         | both             | Fan control during active charging. Battery sends a request (always `0x0064 = 100`), charger replies with the actual setting (ramps 5 → 14 → 20 → 28 → … → 139). Units uncertain — looks like a PWM duty / set-point. |
| `FANOFF`  | `FFONAF`         | both             | Fan off. Sent in the full-battery end-of-session sequence in place of `FAN_ON` (no thermal load to dissipate). Both sides exchange `data=0x0000`. |
| `CHGPCT`  | `TCPGHC`         | both             | State-of-charge percentage. Battery reports actual SOC (e.g. 28 → 29 → 30 % during the capture). Charger always replies `0x0001` — likely an "ack" rather than its own value. |
| `SLEEP_`  | `_PEELS`         | both             | Session terminator. Sent at the end of a full-battery session; bus goes idle afterwards. Both sides exchange `data=0x0000`. |

### Direction is read from the ADC, not inferred

Direction is determined per-frame from the A0 reading (see
[Direction detection](#direction-detection)). The decoder uses
`TOOL-> ` to mean "this frame is the tool's query" and `BATT-> ` for "this
frame is the battery's response" — applied to per-command formatting (cell
index vs. voltage, etc.). No more byte[0]==0 mislabeling for START_ /
ADJUST.

## Tool commands (USB serial)

Default state at boot: auto-ADJUST disabled.

| Key       | Action                                           |
|-----------|--------------------------------------------------|
| `r`       | Sweep `RD_VOL` across all 14 cells               |
| `t`       | Send `RD_TMP` (sensor 0)                         |
| `s`       | Send `RD_SPC`                                    |
| `c`       | Send `RD_CAP`                                    |
| `f`       | Send `RD_FSH` with `Q=0x3800`                    |
| `a`       | Send `ADJUST` with data `0x0000` (matches what real tools send) |
| `x`       | Toggle auto-ADJUST on/off                        |
| `h` / `?` | Print help                                       |

## Observed cadences

### Smart tool — Gen2 battery

Session init runs once (SPC, CAP, FSH read once each), then the tool
rotates indefinitely:

```
RD_TMP × 2 sensors  →  RD_VOL × 14 cells  →  repeat (~3 s/cycle)
```

### Smart tool — Gen1 battery

The full session is rebuilt every cycle (~5 s):

```
ID×2  →  START_  →  ADJUST(tool)  →  ADJUST(batt echo)
     →  RD_SPC  →  RD_CAP  →  RD_FSH
     →  RD_TMP × 2  →  RD_VOL × 14
     →  ~1 s sustained LOW (battery LED flashes through several colors)
     →  next cycle
```

### Charger — Gen2 battery

After init, a steady ~200 ms-per-exchange telemetry loop:

```
(ADDCUR/OUTCUR × 4)  →  CHGPCT  →  FAN_ON  →  repeat
```

## Helpers

- `helpers/decode.py` — offline frame decoder for serial logs
- `helpers/crc_check.py` — verifies Dallas CRC8 against captures
- `helpers/verify_live.py` — live capture verification
- `helpers/analyze_nexus.py` — pattern analysis across captures

> Note: these helpers parse the **legacy human-readable** serial output
> from the original Arduino sketch. The current PlatformIO firmware emits
> NDJSON, so these scripts will need updating before they're useful
> against live captures from the new firmware.

## License

GNU General Public License v3.0 or later (`GPL-3.0-or-later`). See
[LICENSE](LICENSE) for the full text. Source files carry the SPDX
identifier so license scanners pick it up automatically.
