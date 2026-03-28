import ujson

_PATH = "/flash/config.json"

_DEFAULTS = {
    "roll":  {"kp": 1.8, "ki": 0.08, "kd": 0.12},
    "pitch": {"kp": 1.8, "ki": 0.08, "kd": 0.12},
    "yaw":   {"kp": 2.5, "ki": 0.05, "kd": 0.0},
    "roll_offset":  0.0,
    "pitch_offset": 0.0,
    "base_pressure": 101325.0,
}


def load():
    try:
        with open(_PATH, "r") as f:
            data = ujson.load(f)
        # Fill in any missing keys from defaults
        for k, v in _DEFAULTS.items():
            if k not in data:
                data[k] = v
        return data
    except Exception:
        return dict(_DEFAULTS)


def save(cfg):
    try:
        with open(_PATH, "w") as f:
            ujson.dump(cfg, f)
    except Exception:
        pass
