import sys
from collections import Counter

SAMPLE_RATE_HZ = 20_000
US_PER_SAMPLE = 1_000_000 / SAMPLE_RATE_HZ

with open('JustStart82.csv') as f:
    next(f)
    vals = [float(l) for l in f]

THRESH = 1.5
digital = [1 if v > THRESH else 0 for v in vals]

pulses = []
i = 0
while i < len(digital):
    if digital[i] == 0:
        start = i
        while i < len(digital) and digital[i] == 0:
            i += 1
        pulses.append((start, i - start))
    else:
        i += 1

inside = [(s, l) for (s, l) in pulses if 2 <= l <= 30]

hist = Counter(l for _, l in inside)
print("Pulse-length histogram (samples):")
for length in sorted(hist):
    print(f"  {length} samples ({length*US_PER_SAMPLE:.0f}us): {'#'*hist[length]} ({hist[length]})")

print(f"\nTotal LOW pulses inside burst: {len(inside)}")
print(f"Long pulses (>30 samples): {[l for _,l in pulses if l > 30]}")

# Show position of every pulse to find framing
print("\nAll pulses (idx, sample_pos, length_samples, length_us):")
for idx, (s, ll) in enumerate(inside):
    print(f"  {idx:3d}: pos={s:5d} len={ll:2d} ({ll*US_PER_SAMPLE:.0f}us)")

# Treat 25-sample ones as framing — drop them
data_pulses = [(s, l) for (s, l) in inside if l < 20]
print(f"\nData-only pulses ({len(data_pulses)}): excluding framing (>20 samples)")

SPLIT = 8
bits = ['1' if l < SPLIT else '0' for _, l in data_pulses]
bitstr = ''.join(bits)
print(f"\nBit string (short=1, long=0): {bitstr}")
print(f"Length: {len(bitstr)} bits")

def bytes_msb(bs):
    return [int(bs[i:i+8], 2) for i in range(0, len(bs) - 7, 8)]

def bytes_lsb(bs):
    return [int(bs[i:i+8][::-1], 2) for i in range(0, len(bs) - 7, 8)]

def to_ascii(bs):
    return ''.join(chr(b) if 32 <= b < 127 else '.' for b in bs)

m = bytes_msb(bitstr)
l = bytes_lsb(bitstr)
print(f"MSB-first ({len(m)} bytes): " + ' '.join(f'{b:02x}' for b in m))
print(f"  ASCII: {to_ascii(m)}")
print(f"LSB-first ({len(l)} bytes): " + ' '.join(f'{b:02x}' for b in l))
print(f"  ASCII: {to_ascii(l)}")

bitstr_inv = ''.join('0' if b == '1' else '1' for b in bits)
print(f"\nInverted (short=0, long=1): {bitstr_inv}")
m = bytes_msb(bitstr_inv)
l = bytes_lsb(bitstr_inv)
print(f"MSB-first: " + ' '.join(f'{b:02x}' for b in m))
print(f"  ASCII: {to_ascii(m)}")
print(f"LSB-first: " + ' '.join(f'{b:02x}' for b in l))
print(f"  ASCII: {to_ascii(l)}")
