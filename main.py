"""
PyFC — MicroPython flight controller with iNAV MSP server.

Hardware (pyboard STM32F4):
  I2C bus 2: MPU6050 @ 0x68 (Y9=SCL, Y10=SDA), BMP280 @ 0x76
  Timer 5 PWM: X1=M1, X2=M2, X3=M3, X4=M4 (400Hz, 1000-2000µs)
  USB VCP: MSP server → iNAV Configurator at 115200 baud

Arming:
  Connect via VCP and send any MSP packet to wake.
  For safety, motors only spin when armed=True AND throttle > 1050.
  To arm/disarm at runtime, toggle pyb.LED(1) is just a placeholder —
  add an RC channel or button as needed.
"""

import pyb
import utime
import sensors
import config
from msp import MSPServer
from pid import PID

# Motors are optional — board may be tested without ESCs connected
try:
    import motors as motors
    _motors_ok = True
except Exception:
    # Provide a stub so the rest of the code doesn't crash
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
    utime.sleep_ms(200)  # let BMP280 take first sample
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
# MSP server
# ---------------------------------------------------------------------------
vcp = pyb.USB_VCP()
vcp.setinterrupt(-1)  # disable Ctrl-C over VCP so Configurator bytes pass through

def _save(c):
    config.save(c)
    # Rebuild PID objects from updated config
    global pid_roll, pid_pitch, pid_yaw
    pid_roll  = _make_pid('roll')
    pid_pitch = _make_pid('pitch')
    pid_yaw   = _make_pid('yaw')

msp = MSPServer(vcp, sensors, motors, cfg, _save)

# ---------------------------------------------------------------------------
# Flight state
# ---------------------------------------------------------------------------
armed    = False
throttle = 1000   # base throttle µs (1000-2000)
setpoint_roll  = 0.0
setpoint_pitch = 0.0
setpoint_yaw   = 0.0

DT = 0.02  # 50 Hz loop

# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
pyb.LED(3).on()   # green = running

while True:
    loop_start = utime.ticks_ms()

    # 1. Update sensors (reads IMU + baro, runs complementary filter)
    sensors.update()

    # 2. MSP server — non-blocking VCP poll
    msp.update_cycle()
    if vcp.any():
        data = vcp.read(256)
        if data:
            msp.feed(data)

    # 3. Flight control (only when armed)
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

    # 4. Heartbeat LED
    pyb.LED(2).toggle()

    # 5. Hold 20 ms loop (50 Hz)
    elapsed = utime.ticks_diff(utime.ticks_ms(), loop_start)
    remaining = 20 - elapsed
    if remaining > 0:
        utime.sleep_ms(remaining)
