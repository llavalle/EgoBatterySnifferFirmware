"""Decode nexus_dump_20khz.vcd: extract every frame, identify commands."""
import sys
from collections import Counter

VCD = "nexus_dump_20khz.vcd"
US_PER_TICK = 10  # from $timescale

# Parse VCD - extract (time_us, state) transitions
transitions = []
with open(VCD) as f:
    in_data = False
    for line in f:
        line = line.strip()
        if line.startswith("$enddefinitions"):
            in_data = True
            continue
        if not in_data or not line.startswith("#"):
            continue
        # Lines like "#71995 0!" or "#71995" (just timestamp)
        parts = line.split()
        ts = int(parts[0][1:])
        if len(parts) > 1:
            tok = parts[1]
            state = int(tok[0])
            transitions.append((ts * US_PER_TICK, state))
        # else: just a timestamp marker, no value change

print(f"Total transitions: {len(transitions)}")
if not transitions:
    sys.exit(0)

# Build pulse list: each LOW pulse = (start_us, duration_us)
# Find rising/falling edges and measure LOW durations
pulses_low = []
pulses_high = []
for i in range(1, len(transitions)):
    t_prev, s_prev = transitions[i-1]
    t_now,  s_now  = transitions[i]
    duration = t_now - t_prev
    if s_prev == 0 and s_now == 1:
        # LOW pulse just ended
        pulses_low.append((t_prev, duration))
    elif s_prev == 1 and s_now == 0:
        # HIGH gap just ended
        pulses_high.append((t_prev, duration))

# Group LOW pulses into frames separated by long HIGH gaps (>5ms)
FRAME_GAP_US = 5000
frames = []   # list of list of (start_us, low_duration_us)
current = []
last_low_end = 0
for start, dur in pulses_low:
    high_gap_before = start - last_low_end
    if high_gap_before > FRAME_GAP_US and current:
        frames.append(current)
        current = []
    current.append((start, dur))
    last_low_end = start + dur
if current:
    frames.append(current)

print(f"Frames: {len(frames)}")

# Decode each frame
def crc8_dallas(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8C) if (crc & 1) else (crc >> 1)
    return crc

# Bit classification thresholds (us)
def classify(d):
    if d < 100: return None       # glitch
    if d < 400: return 1          # short = bit 1
    if d < 900: return 0          # long  = bit 0
    if d < 2000: return 'sync'
    return 'longLow'

def decode_frame(pulses, idx):
    start_us = pulses[0][0]
    bits = []
    sync_count = 0
    for s, d in pulses:
        c = classify(d)
        if c == 'sync':
            sync_count += 1
            continue
        if c == 'longLow':
            continue
        if c is None:
            continue
        bits.append(c)
    # Optional 1-bit shift for 8N+1 frames
    if len(bits) % 8 == 1:
        bits = bits[1:]
        shifted = True
    else:
        shifted = False
    nbytes = len(bits) // 8
    leftover = len(bits) % 8
    # LSB-first byte assembly
    bs = []
    for i in range(nbytes):
        b = 0
        for j in range(8):
            if bits[i*8 + j]:
                b |= (1 << j)
        bs.append(b)
    return {
        'idx': idx,
        'start_us': start_us,
        'pulses': len(pulses),
        'bits': len(bits),
        'shifted': shifted,
        'sync': sync_count,
        'bytes': bs,
        'leftover': leftover,
    }

def is_cmd(bs, name_reversed):
    if len(bs) < 8: return False
    return bs[2:8] == [ord(c) for c in name_reversed]

def name_for(bs):
    if len(bs) < 8: return None
    cmd_bytes = bs[2:8]
    # reverse to read
    chars = ''.join(chr(b) if 32 <= b < 127 else '?' for b in reversed(cmd_bytes))
    return chars

decoded = [decode_frame(p, i) for i, p in enumerate(frames)]

# Stats
print(f"\nFrame size histogram (bits):")
hist = Counter(f['bits'] for f in decoded)
for k in sorted(hist):
    print(f"  {k}b -> {hist[k]} frames")

# Print all frames in time order with command interpretation
print(f"\nAll frames:")
for f in decoded:
    bs = f['bytes']
    hex_s = ' '.join(f'{b:02x}' for b in bs)
    if f['leftover']:
        hex_s += f"(+{f['leftover']}b)"
    cmd = name_for(bs) if len(bs) == 9 else None
    crc_status = ''
    if len(bs) >= 2 and f['leftover'] == 0:
        expected = crc8_dallas(bs[:-1])
        crc_status = 'OK' if expected == bs[-1] else f'BAD(want {expected:02x})'
    data_val = (bs[1] << 8) | bs[0] if len(bs) >= 2 else 0
    line = f"  [{f['idx']:3d}] @{f['start_us']/1000:.1f}ms  {f['bits']:3d}b  {hex_s:30s} CRC {crc_status:14s}"
    if cmd:
        line += f' cmd="{cmd}" data=0x{data_val:04x}'
    elif len(bs) == 5:
        line += f' [ID]'
    print(line)
