# Gen 1 RD_FSH page sweep — decoding notes

Working notes from sniffing two real Gen 1 packs talking to a Nexus 3000.
Pre-V2 we'd only ever issued a synthetic `Q=0x3800` test probe; this is
the first time we've seen what tools actually do on the wire.

## Protocol observation

On a Gen 1 pack, the tool runs a **two-stage** RD_FSH flow:

1. **Probe at `addr 0x38`.** Battery replies `0x38FF` (the documented Gen 1
   "stub"). This is the tool's signal to switch into Gen 1 mode.
2. **Page sweep `addr 0x78..0x7B`.** Each query carries the page address in
   the high byte of the 16-bit data word; the battery echoes the address
   and returns the byte stored at that flash page in the low byte.

So `RD_FSH` is a generic single-byte flash read keyed on `data.hi = addr`,
not a status-word read. The address being queried tells us what the tool
*wants*, not what generation the pack *is* — that's why gen detection
lives on ADJUST now (see `src/main.cpp` ADJUST branch).

A Gen 2 pack would (per legacy notes) answer the `0x38` probe with a
real status word instead of `0xFF`. We have not yet captured this and
the 0x38xx interpretation remains unverified.

## Page map across captures

|       | 7.5 Ah @ 96 % | 7.5 Ah @ 37 % | 5.0 Ah @ 86 % | 5.0 Ah @ 32 % | Notes |
|-------|---------------|---------------|---------------|---------------|-------|
| 0x38  | 0xFF (stub)   | 0xFF (stub)   | 0xFF (stub)   | 0xFF (stub)   | Gen 1 marker |
| 0x78  | **0x23** (35) | **0x23** (35) | **0x23** (35) | **0x23** (35) | constant — likely fixed identifier |
| 0x79  | **0xF8** (248)| **0xF8** (248)| **0xD0** (208)| **0xD0** (208)| per-pack, stable across SoC |
| 0x7A  | **0x2A** (42) | **0x2A** (42) | **0x2A** (42) | **0x2A** (42) | constant — likely fixed identifier |
| 0x7B  | **0x79** (121)| **0x79** (121)| **0x06** (6)  | **0x06** (6)  | per-pack, stable across SoC |

Source captures (all in `EgoBatterySnifferGui/captures/`):
- `capture_2026-05-14_132118_nexus3000_7.5Ah_96soc_charging_complete_plug.ndjson`
  — 7.5 Ah pack, 96 % SoC, idle (0.00 A), 4.14 V/cell avg
- `capture_2026-05-14_152338_7.5AH_37soc_2.7of_7.5_remain.ndjson`
  — same 7.5 Ah pack after discharge to 37 % SoC, 3.59 V/cell avg
- `capture_2026-05-14_135931_nexus3000_5Ah_86_4.3.ndjson`
  — 5.0 Ah pack, 86 % SoC, 0.11 A load, 73 °F, 4.07 V/cell avg
- `capture_2026-05-14_162112_nexus3000_5ah_32soc_1.6of5.0_remains.ndjson`
  — same 5.0 Ah pack after discharge to 32 % SoC, 3.55 V/cell avg

## Current best guesses

### 0x78 = 0x23, 0x7A = 0x2A — fixed identifiers

Both bytes constant across two different packs of different capacities.
Plausibly model code, protocol version, or BMS firmware identifier.
Won't be confirmed as "fixed" until we see them stay the same on a
*third* differently-built pack.

### 0x79 — likely **learned full-charge capacity per cell, in cAh**

| Pack         | 0x79 raw | / nameplate (250) | implied SoH |
|--------------|----------|-------------------|-------------|
| 7.5 Ah       | 248      | 99.2 %            | mildly aged |
| 5.0 Ah       | 208      | 83.2 %            | well-aged   |

The Nexus very likely uses this to bend the OCV→SoC curve for aged
packs and to drive any "this pack is shot" warnings. Both observed
values are < nameplate (which a healthy degradation model demands) and
are in the same units as `RD_CAP` (cAh/cell), which is a tidy reuse.

**Confirmed: this is a slow-moving / persistent register.** Demonstrated
twice now:
- 7.5 Ah pack: 0x79 stayed at 0xF8 across the 96 % → 37 % discharge.
- 5.0 Ah pack: 0x79 stayed at 0xD0 across the 86 % → 32 % discharge.

So 0x79 is *not* tracking SoC at all — it's the kind of value that only
updates on a full cycle (capacity learning) or never updates at all
(manufacturing constant). Still consistent with the cAh-per-cell-SoH
hypothesis.

**Still not proven.** To distinguish "learned SoH" from "permanent
manufacturing constant", we'd need to either (a) catch 0x79 changing
across multiple full charge/discharge cycles on the same pack over
weeks, or (b) find a fresh-from-factory pack reading 0xFA (= 250 =
nameplate).

### 0x7B — pack-specific, persistent

| Pack         | 0x7B raw |
|--------------|----------|
| 7.5 Ah       | 121      |
| 5.0 Ah       | 6        |

**Confirmed stable across SoC** on both packs:
- 7.5 Ah pack stayed at 0x79 across 96 % → 37 %.
- 5.0 Ah pack stayed at 0x06 across 86 % → 32 %.

That rules out the "recent-discharge / last-DOD telemetry" branch —
0x7B doesn't update on each session.

Remaining possibilities:
- Cycle count (lifetime).
- Pack metadata (build batch, manufacture date encoding).
- Some other slow-moving lifetime stat.

Cycle count is the most plausible practical guess (the Nexus would
care about it). Confirmation needs either a pack we can fully cycle
ourselves while watching the byte, or a known-old vs known-new pair.

## Where SoC actually comes from

Both Nexus displays match `remaining_Ah = nameplate × SoC%` exactly:
- 7.5 Ah × 0.96 = 7.20 Ah ✓
- 5.0 Ah × 0.86 = 4.30 Ah ✓

So the "remaining capacity" the Nexus shows is **derived**, not stored.
SoC itself most plausibly comes from the OCV→SoC curve evaluated on the
RD_VOL cell readings (4.14 V/cell → ~95 %; 4.07 V/cell under light
load → ~85 %). The 0x79 byte may modulate that curve for aged packs but
isn't the *source* of SoC.

## TODOs (in priority order)

- [x] **Re-sniff the same pack at a different SoC.** Done twice:
      96 % → 37 % on the 7.5 Ah pack, 86 % → 32 % on the 5.0 Ah pack.
      Both 0x79 and 0x7B are persistent / SoC-independent on both packs.
- [ ] **Sniff a fresh-from-factory Gen 1 pack** (or any pack whose
      true age/SoH we know). If 0x79 reads near 0xFA = 250 on a young
      pack, the learned-SoH theory is essentially confirmed.
- [ ] **Sniff a third capacity variant** (ideally a 1P = 2.5 Ah pack,
      or anything with different cell count). Confirms 0x78/0x7A as
      truly fixed identifiers rather than coincidentally-equal-on-
      14S-packs values.
- [ ] **Sniff a Gen 2 pack doing RD_FSH.** First real capture of a
      `0x38xx` non-stub response. Lets us finally verify the legacy
      assumption about Gen 2 status bits and the ADJUST `0x00A0`
      marker we currently take on faith from the README.
- [x] **Save the 5.0 Ah pack capture to `captures/`.** Done as
      `capture_2026-05-14_135931_nexus3000_5Ah_86_4.3.ndjson`.
- [ ] **Add the page map to the GUI's debug pane** so live captures
      surface 0x78/0x79/0x7A/0x7B values as they're learned, without
      grepping NDJSON. (Held off pending more samples — header should
      probably show *learned* SoH rather than raw 0x79 once we trust
      the interpretation.)
- [ ] **Re-verify the 0x00A0 Gen 2 ADJUST marker** when we get a Gen 2
      sniff. The firmware comment in `main.cpp` ADJUST block already
      flags this as needing confirmation.
- [ ] **Long-running watch on a pack we cycle ourselves** to see if
      0x79 (and/or 0x7B) ever moves. Would distinguish "permanent
      manufacturing constant" from "slowly-learned SoH/cycle count".

## What we changed in code based on this

- `src/main.cpp` RD_FSH branch: emits `fsh_addr`/`fsh_value` per
  page; no longer guesses `gen` from the response.
- `src/main.cpp` ADJUST branch: emits `gen` (1 on `0x0000`, 2 on
  `0x00A0`) when the frame is from the battery.
- `EgoBatterySnifferGui/ego_gui/state.py`: new
  `BatteryState.fsh_pages: dict[int, int]`. ADJUST handler updates
  `gen`. Legacy captures (no `fsh_addr` field) are back-filled from
  the `status` field on replay.
