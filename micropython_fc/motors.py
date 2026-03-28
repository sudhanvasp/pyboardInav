import pyb

# Timer 5 at 400 Hz, channels 1-4 on pins X1-X4
_tim = pyb.Timer(5, freq=400)
_ch = [
    _tim.channel(1, pyb.Timer.PWM, pin=pyb.Pin("X1")),
    _tim.channel(2, pyb.Timer.PWM, pin=pyb.Pin("X2")),
    _tim.channel(3, pyb.Timer.PWM, pin=pyb.Pin("X3")),
    _tim.channel(4, pyb.Timer.PWM, pin=pyb.Pin("X4")),
]

# Current motor outputs (1000-2000 us), exposed for MSP_MOTOR
outputs = [1000, 1000, 1000, 1000]


def _clamp(v):
    return max(1000, min(2000, int(v)))


def set_motors(s1, s2, s3, s4):
    outputs[0] = _clamp(s1)
    outputs[1] = _clamp(s2)
    outputs[2] = _clamp(s3)
    outputs[3] = _clamp(s4)
    for i, ch in enumerate(_ch):
        ch.pulse_width(outputs[i])


def disarm():
    """Set all motors to minimum throttle."""
    set_motors(1000, 1000, 1000, 1000)


def mix(throttle, roll_corr, pitch_corr, yaw_corr):
    """
    X-frame mixing:
      M1 front-right: +roll -pitch +yaw
      M2 rear-right:  +roll +pitch -yaw
      M3 rear-left:   -roll +pitch +yaw
      M4 front-left:  -roll -pitch -yaw
    """
    set_motors(
        throttle + roll_corr - pitch_corr + yaw_corr,
        throttle + roll_corr + pitch_corr - yaw_corr,
        throttle - roll_corr + pitch_corr + yaw_corr,
        throttle - roll_corr - pitch_corr - yaw_corr,
    )
