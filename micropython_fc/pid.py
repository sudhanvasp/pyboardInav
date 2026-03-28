class PID:
    def __init__(self, kp, ki, kd, i_limit=150.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = i_limit
        self._iterm = 0.0
        self._last_error = 0.0

    def update(self, error, dt):
        self._iterm += error * self.ki * dt
        if self._iterm > self.i_limit:
            self._iterm = self.i_limit
        elif self._iterm < -self.i_limit:
            self._iterm = -self.i_limit
        d_out = ((error - self._last_error) / dt) * self.kd if dt > 0 else 0.0
        self._last_error = error
        return error * self.kp + self._iterm + d_out

    def reset(self):
        self._iterm = 0.0
        self._last_error = 0.0
