from machine import I2C
import math
import utime

MPU_ADDR = 0x68
BMP_ADDR = 0x76

# Shared I2C bus 2 (Y9=SCL, Y10=SDA)
i2c = I2C(2, freq=400000)

# --- State ---
roll = 0.0
pitch = 0.0
yaw = 0.0

ax = ay = az = 0.0   # G
gx = gy = gz = 0.0   # dps

pressure_pa = 101325.0
temperature_c = 25.0
altitude_m = 0.0

_base_pressure = 101325.0
_alpha = 0.96
_last_us = 0

# BMP280 compensation values (loaded once at init)
_bmp_dig = {}


def _s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v > 32767 else v


def _u16(hi, lo):
    return (hi << 8) | lo


# ---------------------------------------------------------------------------
# MPU-6050
# ---------------------------------------------------------------------------
def _mpu_init():
    i2c.writeto_mem(MPU_ADDR, 0x6B, b'\x00')  # wake
    i2c.writeto_mem(MPU_ADDR, 0x1B, b'\x00')  # gyro  ±250 dps  → 131 LSB/dps
    i2c.writeto_mem(MPU_ADDR, 0x1C, b'\x00')  # accel ±2 g      → 16384 LSB/g


def _mpu_read():
    global ax, ay, az, gx, gy, gz
    d = i2c.readfrom_mem(MPU_ADDR, 0x3B, 14)
    ax = _s16(d[0],  d[1])  / 16384.0
    ay = _s16(d[2],  d[3])  / 16384.0
    az = _s16(d[4],  d[5])  / 16384.0
    # d[6:8] = temperature, skip
    gx = _s16(d[8],  d[9])  / 131.0
    gy = _s16(d[10], d[11]) / 131.0
    gz = _s16(d[12], d[13]) / 131.0


# ---------------------------------------------------------------------------
# BMP280
# ---------------------------------------------------------------------------
def _bmp_read_calib():
    """Read factory compensation coefficients (registers 0x88-0x9F)."""
    c = i2c.readfrom_mem(BMP_ADDR, 0x88, 24)
    _bmp_dig['T1'] = _u16(c[1],  c[0])
    _bmp_dig['T2'] = _s16(c[3],  c[2])
    _bmp_dig['T3'] = _s16(c[5],  c[4])
    _bmp_dig['P1'] = _u16(c[7],  c[6])
    _bmp_dig['P2'] = _s16(c[9],  c[8])
    _bmp_dig['P3'] = _s16(c[11], c[10])
    _bmp_dig['P4'] = _s16(c[13], c[12])
    _bmp_dig['P5'] = _s16(c[15], c[14])
    _bmp_dig['P6'] = _s16(c[17], c[16])
    _bmp_dig['P7'] = _s16(c[19], c[18])
    _bmp_dig['P8'] = _s16(c[21], c[20])
    _bmp_dig['P9'] = _s16(c[23], c[22])


def _bmp_init():
    # Normal mode, pressure OS×4, temperature OS×1, standby 0.5ms
    i2c.writeto_mem(BMP_ADDR, 0xF4, b'\x6B')  # 0110_1011
    i2c.writeto_mem(BMP_ADDR, 0xF5, b'\x00')
    _bmp_read_calib()


def _bmp_read():
    global pressure_pa, temperature_c, altitude_m
    raw = i2c.readfrom_mem(BMP_ADDR, 0xF7, 6)
    adc_p = ((raw[0] << 12) | (raw[1] << 4) | (raw[2] >> 4))
    adc_t = ((raw[3] << 12) | (raw[4] << 4) | (raw[5] >> 4))

    # Temperature compensation (BMP280 datasheet 4.2.3)
    d = _bmp_dig
    var1 = (adc_t / 16384.0 - d['T1'] / 1024.0) * d['T2']
    var2 = (adc_t / 131072.0 - d['T1'] / 8192.0) ** 2 * d['T3']
    t_fine = var1 + var2
    temperature_c = t_fine / 5120.0

    # Pressure compensation
    var1 = t_fine / 2.0 - 64000.0
    var2 = var1 * var1 * d['P6'] / 32768.0
    var2 += var1 * d['P5'] * 2.0
    var2 = var2 / 4.0 + d['P4'] * 65536.0
    var1 = (d['P3'] * var1 * var1 / 524288.0 + d['P2'] * var1) / 524288.0
    var1 = (1.0 + var1 / 32768.0) * d['P1']
    if var1 == 0:
        pressure_pa = 101325.0
        return
    p = 1048576.0 - adc_p
    p = ((p - var2 / 4096.0) * 6250.0) / var1
    var1 = d['P9'] * p * p / 2147483648.0
    var2 = p * d['P8'] / 32768.0
    pressure_pa = p + (var1 + var2 + d['P7']) / 16.0

    altitude_m = (_base_pressure - pressure_pa) * 0.085


# ---------------------------------------------------------------------------
# Complementary filter
# ---------------------------------------------------------------------------
def _comp_filter(dt):
    global roll, pitch, yaw
    a_r = math.degrees(math.atan2(ay, az))
    a_p = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
    roll  = _alpha * (roll  + gx * dt) + (1.0 - _alpha) * a_r
    pitch = _alpha * (pitch + gy * dt) + (1.0 - _alpha) * a_p
    yaw  += gz * dt
    if yaw > 360.0:
        yaw -= 360.0
    elif yaw < 0.0:
        yaw += 360.0


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------
def init(base_pressure=None):
    global _last_us, _base_pressure
    _mpu_init()
    try:
        _bmp_init()
        if base_pressure is not None:
            _base_pressure = base_pressure
    except Exception:
        pass
    _last_us = utime.ticks_us()


def update():
    global _last_us
    now = utime.ticks_us()
    dt = utime.ticks_diff(now, _last_us) / 1_000_000.0
    _last_us = now

    try:
        _mpu_read()
    except Exception:
        return
    try:
        _bmp_read()
    except Exception:
        pass

    _comp_filter(dt)


def set_base_pressure(p):
    global _base_pressure
    _base_pressure = p
