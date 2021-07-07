map = [
  # Control Outputs (13, 14, 15, 16, 17, 18, 19)
  0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19,
  # Dither, Random (2C, 2D)
  0x2C, 0x2D,
  # Antenna RX1 (29, 2A, 2B)
  0x29, 0x2A, 0x2B,
  # Antenna TX (1A, 1B, 1F)
  0x1A, 0x1B, 0x1F,
  # Bypass all HPFs (2E)
  0x2E,
  # 6m low noise amplifier (2F)
  0x2F,
  # Disable T/R relay (32)
  0x32,
  # ATT1 (PA3, PA4, PA5, PA6, PA7)
  0x53, 0x54, 0x55, 0x56, 0x57,
  # ATT2 (PA15, PB3, PB4, PB5, PB6)
  0x5F, 0x63, 0x64, 0x65, 0x66,
  # BCD code for RX1 band (34, 35, 36, 37)
  0x34, 0x35, 0x36, 0x37,
  # BCD code for RX2 band (38, 39, 3A, 3B)
  0x38, 0x39, 0x3A, 0x3B,
  # BPF TX (44, 47, 42, 43, 41, 40, 3C, 3D, 3E, 3F, 46, 45)
  0x44, 0x47, 0x42, 0x43, 0x41, 0x40, 0x3C, 0x3D, 0x3E, 0x3F, 0x46, 0x45,
  # BPF RX1 (0A, 09, 08, 04, 05, 06, 07, 03, 02, 01, 00, 0B)
  0x0A, 0x09, 0x08, 0x04, 0x05, 0x06, 0x07, 0x03, 0x02, 0x01, 0x00, 0x0B,
  # BPF RX2 (28, 26, 27, 22, 23, 21, 20, 1C, 1D, 1E, 25, 24)
  0x28, 0x26, 0x27, 0x22, 0x23, 0x21, 0x20, 0x1C, 0x1D, 0x1E, 0x25, 0x24,
  # LPF TX (0C, 0D, 0E, 0F, 10, 11, 12)
  0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12,
  # TX band != RX1 band (33)
  0x33,
  # Relays (30, 31)
  0x30, 0x31,
  # PTT (48)
  0x48,
  # Preamps (49, 4A)
  0x49, 0x4A,
  # Antenna RX2 (4B, 4C)
  0x4B, 0x4C]

result = []

for i in range(len(map)):
  reg = map[i] >> 4
  idx = i >> 5
  src = (i & 0x1F) - (map[i] & 0xF)
  dst = map[i] & 0xF
  result.append((reg, idx, src, dst))

result.sort()

result.append((-1, -1, -1, -1))

reg, idx, src, dst = result[0]
line = ''
mask = 0

for e in result:
  mask |= 1 << dst
  if e[1] != idx or e[2] != src or abs(e[3] - dst) > 1:
    if src < 0:
      shift = -src
      op = '<<'
    else:
      shift = src
      op = '>>'
    if len(line) > 0:
      line += ' | '
    line += '((data[%d] %s %d) & 0x%04x)' % (idx, op, shift, mask)
    mask = 0
  if e[0] != reg:
    print('  bits[%d] = %s;' % (reg, line))
    line = ''
  reg, idx, src, dst = e
