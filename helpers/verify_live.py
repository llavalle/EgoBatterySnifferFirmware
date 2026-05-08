def crc8_dallas(data, poly=0x8C):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ poly if (crc & 1) else (crc >> 1)
    return crc

# Live decode (short=0/long=1, LSB-first per findings)
live_5  = [0x55, 0xF4, 0xB5, 0xB7, 0x74]
live_9  = [0xFF, 0xFF, 0x41, 0x57, 0x5B, 0x7D, 0x57, 0x59, 0x9F]

# Inverted (short=1/long=0, LSB-first - what our Python decode.py used)
inv_5 = [(~b) & 0xff for b in live_5]
inv_9 = [(~b) & 0xff for b in live_9]

print("=== 5-byte beacon ===")
print(f"Live (short=0):     {' '.join(f'{b:02x}' for b in live_5)}")
print(f"  CRC over [0..3] = 0x{crc8_dallas(live_5[:4]):02x}, expected = 0x{live_5[4]:02x}, match? {crc8_dallas(live_5[:4]) == live_5[4]}")
print(f"Inverted (short=1): {' '.join(f'{b:02x}' for b in inv_5)}")
print(f"  CRC over [0..3] = 0x{crc8_dallas(inv_5[:4]):02x}, expected = 0x{inv_5[4]:02x}, match? {crc8_dallas(inv_5[:4]) == inv_5[4]}")

print("\n=== 9-byte message ===")
print(f"Live (short=0):     {' '.join(f'{b:02x}' for b in live_9)}")
print(f"  CRC over [0..7] = 0x{crc8_dallas(live_9[:8]):02x}, expected = 0x{live_9[8]:02x}, match? {crc8_dallas(live_9[:8]) == live_9[8]}")
print(f"Inverted (short=1): {' '.join(f'{b:02x}' for b in inv_9)}")
print(f"  CRC over [0..7] = 0x{crc8_dallas(inv_9[:8]):02x}, expected = 0x{inv_9[8]:02x}, match? {crc8_dallas(inv_9[:8]) == inv_9[8]}")

# Try byte-reversed command name interpretations
print("\n=== Inverted 9-byte: command name (reversed bytes 2..7) ===")
cmd = ''.join(chr(b) if 32 <= b < 127 else '.' for b in reversed(inv_9[2:8]))
print(f"  cmd = '{cmd}'")
print(f"  data uint16 LE @ wire[0..1] = 0x{inv_9[1]:02x}{inv_9[0]:02x}")
