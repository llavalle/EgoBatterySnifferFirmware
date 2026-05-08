# EGOBatteryCommunication

Arduino Mega sketch that decodes the 1-wire serial protocol on an EGO power-tool
battery's `D` pin. Captures live tool↔battery exchanges, prints decoded frames
over USB serial, and can also act as a minimal tool by transmitting commands.

## Hardware wiring

```
Mega 5V --[470 ohm]--+-- D pin
                     +-- Pin 2   (digital — edge detection / TX)
                     +-- A0      (analog — direction detection)
Mega GND ----------- - pin
```

Pin 2 is `INPUT`; the line idles HIGH at ~4.0 V via the 470 Ω pull-up to 5 V,
matching the level real EGO tools drive. The Arduino pulls LOW via
`OUTPUT/LOW` only during TX. **Never drive `OUTPUT HIGH`** — the resistor
handles HIGH.

A0 taps the same node and is sampled at the start of each frame to determine
who's driving the line (see [Direction detection](#direction-detection)).

### Passive sniffing
For passive sniffing of a real tool↔battery conversation, **remove the 470 Ω
pull-up** (or replace with ≥10 kΩ) so we don't load the bus, and disable any
TX path (auto-ADJUST is off by default).

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
tool    →  START_  data=0x00D7                (charger identifier — non-zero!)
battery →  START_  data=0x0000                (re-issued)
tool    →  START_  data=0x00D7                (re-issued)
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
  `ADDCUR=2`, OUTCUR's rate of climb slows — the BMS is fine-tuning. The
  charger doesn't decide how much to push; the battery does.
- **Charger never reads cells / temps.** No `RD_VOL`, `RD_TMP`, `RD_SPC`,
  `RD_CAP`, or `RD_FSH` in the entire charge session — the charger trusts
  the BMS via `ADDCUR` and `CHGPCT`.
- **`CHGPCT` reports SOC.** Battery sends actual percentage in real time
  (28 → 29 → 30 % observed). Charger always replies `0x0001` (ack/seen).
- **`START_` from the charger has data `0x00D7`** — the only place we've
  seen a non-zero `START_` payload. Possibly a charger identifier or
  capability code.

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
| `START_`  | `_TRATS`         | **batt → tool** *and* **tool → batt** | Battery sends `0x0000` (same as diagnostic). Charger replies with its own `START_`, data `0x00D7` (215) — first observed non-zero `START_` payload, likely a charger identifier. |
| `EVACHG`  | `GHCAVE`         | both             | "Enter charge mode" (electric-vehicle-adjust-charge?). Both sides exchange `data=0x0403` (1027). Looks like an explicit mode marker. |
| `GETCG1` / `OUTCG1` | `1GCTEG` / `1GCTUO` | batt / tool | "Charge stage 1" query/reply. Both `data=0x0000` in observed sample. |
| `GET_VM` / `OUT_VM` | `MV_TEG` / `MV_TUO` | batt / tool | Battery asks the charger's output voltage. `OUT_VM` data is centivolts (e.g. `0x1770 = 6000` → 60.00 V). |
| `GET_CM` / `OUT_CM` | `MC_TEG` / `MC_TUO` | batt / tool | Battery asks the charger's output current. `OUT_CM` data is centiamps (e.g. `0x012C = 300` → 3.00 A). |
| `EN_OUT`  | `TUO_NE`         | tool → batt      | Charger toggles its output: `data=0x0001` enables, `0x0000` disables. |
| `CHGING`  | `GNIGHC`         | both             | "Charging" heartbeat / handshake. Data `0x0000` both sides. |
| `ADDCUR`  | `RUCDDA`         | batt → tool      | Battery requests an increment to charge current (delta in unknown unit; observed values 2 and 5). Drives the closed-loop ramp. |
| `OUTCUR`  | `RUCTUO`         | tool → batt      | Charger reports actual current it's delivering. Increases monotonically while battery sends `ADDCUR>0`; rate of increase tracks `ADDCUR` magnitude. |
| `FAN_ON`  | `NO_NAF`         | both             | Fan control. Battery sends a request (always `0x0064 = 100`), charger replies with the actual setting (ramps 5 → 14 → 20 → 28 → … → 139). Units uncertain — looks like a PWM duty / set-point. |
| `CHGPCT`  | `TCPGHC`         | both             | State-of-charge percentage. Battery reports actual SOC (e.g. 28 → 29 → 30 % during the capture). Charger always replies `0x0001` — likely an "ack" rather than its own value. |

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

- `Helpers/decode.py` — offline frame decoder for serial logs
- `Helpers/crc_check.py` — verifies Dallas CRC8 against captures
- `Helpers/verify_live.py` — live capture verification
- `Helpers/analyze_nexus.py` — pattern analysis across captures
