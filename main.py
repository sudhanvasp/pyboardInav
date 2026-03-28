"""
PyFC — MicroPython flight controller with iNAV MSP server.

Hardware (pyboard STM32F4):
  I2C bus 2: MPU6050 @ 0x68 (Y9=SCL, Y10=SDA), BMP280 @ 0x76
  Timer 5 PWM: X1=M1, X2=M2, X3=M3, X4=M4 (400Hz, 1000-2000µs)
  UART 6: Y2=CRSF RX (ELRS receiver), Y1=CRSF TX (telemetry, optional)
  USB VCP: MSP server → iNAV Configurator at 115200 baud

RC channel mapping (CRSF/ELRS):
  CH1 (idx 0): Roll     — ±MAX_ANGLE degrees
  CH2 (idx 1): Pitch    — ±MAX_ANGLE degrees
  CH3 (idx 2): Throttle — 1000-2000 µs direct
  CH4 (idx 3): Yaw rate — ±MAX_YAW_RATE deg/s
  CH5 (idx 4): Arm      — >1700 = armed, <1300 = disarmed

Arming:
  Arm:   CH5 > 1700 AND throttle < 1100 AND CRSF link healthy
  Disarm: CH5 < 1300 OR CRSF link lost (>200 ms silence)
"""

import pyb
import utime
from machine import UART
import sensors
import config
from msp import MSPServer
from pid import PID
from crsf import CRSFReceiver

# ---------------------------------------------------------------------------
# RC parameters
# ---------------------------------------------------------------------------
MAX_ANGLE    = 30.0   # degrees max roll/pitch command
MAX_YAW_RATE = 90.0   # degrees/sec max yaw rate command
DEADBAND     = 20     # µs deadband around 1500 on roll/pitch/yaw sticks

# ---------------------------------------------------------------------------
# Motors (optional — board may be tested without ESCs connected)
# ---------------------------------------------------------------------------
try:
    import motors as motors
    _motors_ok = True
except Exception:
    class _MotorStub:
        outputs = [1000, 1000, 1000, 1000]
        def mix(self, *a): pass
        def disarm(self): pass
    motors = _MotorStub()
    _motors_ok = False

# ---------------------------------------------------------------------------
# Load persistent config
# ---------------------------------------------------------------------------
cfg = config.load()

# ---------------------------------------------------------------------------
# Init sensors (non-fatal — MSP server must start regardless)
# ---------------------------------------------------------------------------
try:
    sensors.init(base_pressure=cfg.get('base_pressure', 101325.0))
    utime.sleep_ms(200)
except Exception:
    pass

# ---------------------------------------------------------------------------
# PID controllers (roll, pitch, yaw)
# ---------------------------------------------------------------------------
def _make_pid(axis):
    a = cfg[axis]
    return PID(a['kp'], a['ki'], a['kd'])

pid_roll  = _make_pid('roll')
pid_pitch = _make_pid('pitch')
pid_yaw   = _make_pid('yaw')

# ---------------------------------------------------------------------------
# ELRS / CRSF receiver — UART chosen by Ports config (RX_SERIAL = 64)
# ---------------------------------------------------------------------------
# Maps iNAV port identifier → pyboard UART number
# USART1(0)=X9/X10, USART2(1)=Y5/Y6, USART6(5)=Y1/Y2
_INAV_ID_TO_UART = {0: 1, 1: 2, 5: 6}
_FUNCTION_RX_SERIAL = 64

_uart_elrs = None
_elrs_ok   = False
elrs       = CRSFReceiver()

for _port in cfg.get('ports', []):
    if _port[1] & _FUNCTION_RX_SERIAL:
        _uart_num = _INAV_ID_TO_UART.get(_port[0])
        if _uart_num:
            try:
                _uart_elrs = UART(_uart_num, baudrate=420000, bits=8,
                                  parity=None, stop=1, read_buf_len=128)
                _elrs_ok = True
            except Exception:
                pass
        break

# ---------------------------------------------------------------------------
# MSP server (USB VCP)
# ---------------------------------------------------------------------------
vcp = pyb.USB_VCP()
vcp.setinterrupt(-1)   # disable Ctrl-C over VCP so Configurator bytes pass through

def _save(c):
    config.save(c)
    global pid_roll, pid_pitch, pid_yaw
    pid_roll  = _make_pid('roll')
    pid_pitch = _make_pid('pitch')
    pid_yaw   = _make_pid('yaw')

msp = MSPServer(vcp, sensors, motors, cfg, _save)

# ---------------------------------------------------------------------------
# Flight state
# ---------------------------------------------------------------------------
armed          = False
throttle       = 1000
setpoint_roll  = 0.0
setpoint_pitch = 0.0
setpoint_yaw   = 0.0

DT = 0.02   # 50 Hz loop

def _apply_deadband(val, centre=1500, db=DEADBAND):
    err = val - centre
    if abs(err) < db:
        return 0
    return err - (db if err > 0 else -db)


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
pyb.LED(3).on()   # green = running

while True:
    loop_start = utime.ticks_ms()

    # 1. Update sensors (IMU + baro + complementary filter)
    sensors.update()

    # 2. Read ELRS / CRSF
    if _elrs_ok and _uart_elrs.any():
        elrs.feed(_uart_elrs.read())
    elrs.check_timeout(loop_start)

    # 3. Map RC channels → flight setpoints
    if elrs.healthy:
        throttle = elrs.channels[2]   # CH3: 1000-2000 µs

        roll_stick  = _apply_deadband(elrs.channels[0])
        pitch_stick = _apply_deadband(elrs.channels[1])
        yaw_stick   = _apply_deadband(elrs.channels[3])

        setpoint_roll  = roll_stick  / (500.0 - DEADBAND) * MAX_ANGLE
        setpoint_pitch = pitch_stick / (500.0 - DEADBAND) * MAX_ANGLE

        # Yaw: rate mode — accumulate heading setpoint
        yaw_rate = yaw_stick / (500.0 - DEADBAND) * MAX_YAW_RATE
        setpoint_yaw += yaw_rate * DT
        setpoint_yaw  = setpoint_yaw % 360.0

        # Arm/disarm via CH5
        if elrs.armed and throttle < 1100 and not armed:
            armed = True
            pyb.LED(1).on()
        elif not elrs.armed and armed:
            armed = False
            pyb.LED(1).off()
    else:
        # No RC link — disarm immediately
        if armed:
            armed = False
            pyb.LED(1).off()
        throttle = 1000

    # 4. MSP server — non-blocking VCP poll
    msp.update_cycle()
    if vcp.any():
        data = vcp.read(256)
        if data:
            msp.feed(data)

    # 5. Flight control (only when armed)
    if armed and throttle > 1050:
        roll_corr  = pid_roll.update(setpoint_roll  - sensors.roll,  DT)
        pitch_corr = pid_pitch.update(setpoint_pitch - sensors.pitch, DT)
        yaw_corr   = pid_yaw.update(setpoint_yaw   - sensors.yaw,   DT)
        motors.mix(throttle, roll_corr, pitch_corr, yaw_corr)
    else:
        pid_roll.reset()
        pid_pitch.reset()
        pid_yaw.reset()
        motors.disarm()

    # 6. Heartbeat LED
    pyb.LED(2).toggle()

    # 7. Hold 20 ms loop (50 Hz)
    elapsed   = utime.ticks_diff(utime.ticks_ms(), loop_start)
    remaining = 20 - elapsed
    if remaining > 0:
        utime.sleep_ms(remaining)
