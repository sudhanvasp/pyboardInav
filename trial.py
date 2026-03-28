"""
trial.py — Sensor diagnostic for MPU6050 + BMP280 on I2C bus 2 (Y9/Y10).
Copy to PYBFLASH and run from REPL: exec(open('trial.py').read())
Or rename to main.py temporarily.

Output every 500ms:
  ACC  ax ay az (G)
  GYRO gx gy gz (deg/s)
  BARO pressure (Pa), temp (C), alt (m relative)
"""

import pyb
from machine import I2C
import utime
import math
import struct

i2c = I2C(2, freq=400000)

# ── Scan ──────────────────────────────────────────────────────────────────────
print("Scanning I2C bus 2...")
found = i2c.scan()
print("Devices found:", [hex(a) for a in found])
MPU_OK  = 0x68 in found
BMP_OK  = 0x76 in found or 0x77 in found
BMP_ADDR = 0x76 if 0x76 in found else 0x77

print("MPU6050 @ 0x68:", "YES" if MPU_OK  else "NOT FOUND")
print("BMP280  @ 0x%02x:" % BMP_ADDR, "YES" if BMP_OK else "NOT FOUND")
print()

# ── MPU-6050 init ─────────────────────────────────────────────────────────────
if MPU_OK:
    i2c.writeto_mem(0x68, 0x6B, b'\x00')   # wake
    i2c.writeto_mem(0x68, 0x1B, b'\x00')   # gyro  ±250 dps
    i2c.writeto_mem(0x68, 0x1C, b'\x00')   # accel ±2 g
    print("MPU6050 initialised")

# ── BMP280 init + calibration ─────────────────────────────────────────────────
dig = {}
base_pa = None

def _s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v > 32767 else v

def _u16(hi, lo):
    return (hi << 8) | lo

if BMP_OK:
    # normal mode, pressure OS×4, temp OS×1
    i2c.writeto_mem(BMP_ADDR, 0xF4, b'\x6B')
    i2c.writeto_mem(BMP_ADDR, 0xF5, b'\x00')
    c = i2c.readfrom_mem(BMP_ADDR, 0x88, 24)
    dig['T1'] = _u16(c[1],  c[0])
    dig['T2'] = _s16(c[3],  c[2])
    dig['T3'] = _s16(c[5],  c[4])
    dig['P1'] = _u16(c[7],  c[6])
    dig['P2'] = _s16(c[9],  c[8])
    dig['P3'] = _s16(c[11], c[10])
    dig['P4'] = _s16(c[13], c[12])
    dig['P5'] = _s16(c[15], c[14])
    dig['P6'] = _s16(c[17], c[16])
    dig['P7'] = _s16(c[19], c[18])
    dig['P8'] = _s16(c[21], c[20])
    dig['P9'] = _s16(c[23], c[22])
    utime.sleep_ms(100)
    print("BMP280 initialised, reading base pressure...")

def bmp_read():
    raw = i2c.readfrom_mem(BMP_ADDR, 0xF7, 6)
    adc_p = (raw[0] << 12) | (raw[1] << 4) | (raw[2] >> 4)
    adc_t = (raw[3] << 12) | (raw[4] << 4) | (raw[5] >> 4)
    var1 = (adc_t / 16384.0 - dig['T1'] / 1024.0) * dig['T2']
    var2 = (adc_t / 131072.0 - dig['T1'] / 8192.0) ** 2 * dig['T3']
    t_fine = var1 + var2
    temp = t_fine / 5120.0
    var1 = t_fine / 2.0 - 64000.0
    var2 = var1 * var1 * dig['P6'] / 32768.0 + var1 * dig['P5'] * 2.0
    var2 = var2 / 4.0 + dig['P4'] * 65536.0
    var1 = (dig['P3'] * var1 * var1 / 524288.0 + dig['P2'] * var1) / 524288.0
    var1 = (1.0 + var1 / 32768.0) * dig['P1']
    if var1 == 0:
        return None, temp
    p = 1048576.0 - adc_p
    p = ((p - var2 / 4096.0) * 6250.0) / var1
    var1 = dig['P9'] * p * p / 2147483648.0
    var2 = p * dig['P8'] / 32768.0
    p += (var1 + var2 + dig['P7']) / 16.0
    return p, temp

if BMP_OK:
    base_pa, _ = bmp_read()
    print("Base pressure: %.1f Pa  (%.1f hPa)" % (base_pa, base_pa / 100))
    print()

# ── Main loop ─────────────────────────────────────────────────────────────────
print("Reading every 500ms — Ctrl-C to stop\n")
led = pyb.LED(2)
n = 0

while True:
    led.toggle()
    lines = []

    # MPU-6050
    if MPU_OK:
        d = i2c.readfrom_mem(0x68, 0x3B, 14)
        def s16(h, l):
            v = (h << 8) | l
            return v - 65536 if v > 32767 else v
        ax = s16(d[0],  d[1])  / 16384.0
        ay = s16(d[2],  d[3])  / 16384.0
        az = s16(d[4],  d[5])  / 16384.0
        gx = s16(d[8],  d[9])  / 131.0
        gy = s16(d[10], d[11]) / 131.0
        gz = s16(d[12], d[13]) / 131.0
        roll  = math.degrees(math.atan2(ay, az))
        pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
        lines.append("ACC  ax=%+.3f  ay=%+.3f  az=%+.3f G" % (ax, ay, az))
        lines.append("GYRO gx=%+6.1f  gy=%+6.1f  gz=%+6.1f deg/s" % (gx, gy, gz))
        lines.append("ANGLE  roll=%+6.1f  pitch=%+6.1f deg" % (roll, pitch))
    else:
        lines.append("ACC  -- MPU6050 not found --")

    # BMP280
    if BMP_OK:
        pa, tc = bmp_read()
        if pa and base_pa:
            alt = (base_pa - pa) * 0.085
            lines.append("BARO  %.1f Pa  %.1f hPa  %.2f°C  alt=%.2f m" % (pa, pa/100, tc, alt))
        else:
            lines.append("BARO  read error")
    else:
        lines.append("BARO  -- BMP280 not found --")

    print("[%3d]" % n)
    for l in lines:
        print("  ", l)
    print()

    n += 1
    utime.sleep_ms(500)
