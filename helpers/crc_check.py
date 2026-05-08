def crc8_dallas(data, poly=0x8C, init=0x00):
    # Dallas 1-Wire CRC8: poly 0x31, refin=true, refout=true → reflected poly = 0x8C
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ poly
            else:
                crc >>= 1
    return crc

def crc8_generic(data, poly, init=0x00, refin=False, refout=False, xorout=0x00):
    def reflect(v, w):
        r = 0
        for i in range(w):
            if v & (1 << i):
                r |= 1 << (w - 1 - i)
        return r
    crc = init
    for b in data:
        if refin:
            b = reflect(b, 8)
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xff
            else:
                crc = (crc << 1) & 0xff
    if refout:
        crc = reflect(crc, 8)
    return crc ^ xorout

beacon = [0x55, 0xd0, 0x52, 0x62, 0x29]

print("=== Trying CRC over different prefixes (MSB-first beacon: 55 d0 52 62 | 29) ===")
for length in range(1, 5):
    data = beacon[:length]
    expected = beacon[length] if length < 5 else None
    print(f"\nData = {' '.join(f'{b:02x}' for b in data)}, expected next byte = 0x{beacon[length]:02x}" if length < 5 else "")
    print(f"  Dallas 1-Wire CRC8 (poly=0x31 reflected):     0x{crc8_dallas(data):02x}")
    print(f"  CRC8 poly 0x31 init 0x00 (no reflect):        0x{crc8_generic(data, 0x31):02x}")
    print(f"  CRC8 poly 0x07 init 0x00 (CCITT/SMBus-ish):   0x{crc8_generic(data, 0x07):02x}")
    print(f"  CRC8 poly 0x07 init 0xff:                     0x{crc8_generic(data, 0x07, init=0xff):02x}")
    print(f"  CRC8 poly 0x1d (AUTOSAR):                     0x{crc8_generic(data, 0x1d):02x}")
    print(f"  Sum mod 256:                                  0x{sum(data) & 0xff:02x}")
    print(f"  XOR of bytes:                                 0x{0 if not data else __import__('functools').reduce(lambda a,b:a^b, data):02x}")

# Also try LSB-first version
beacon_lsb = [0xaa, 0x0b, 0x4a, 0x46, 0x94]
print("\n\n=== LSB-first beacon: aa 0b 4a 46 | 94 ===")
for length in range(1, 5):
    data = beacon_lsb[:length]
    print(f"\nData = {' '.join(f'{b:02x}' for b in data)}, expected next byte = 0x{beacon_lsb[length]:02x}")
    print(f"  Dallas 1-Wire CRC8:                           0x{crc8_dallas(data):02x}")
    print(f"  CRC8 poly 0x31 init 0x00:                     0x{crc8_generic(data, 0x31):02x}")
    print(f"  Sum mod 256:                                  0x{sum(data) & 0xff:02x}")
