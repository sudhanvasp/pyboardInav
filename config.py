import ujson

_PATH = "/flash/config.json"

_DEFAULTS = {
    "roll":  {"kp": 1.8, "ki": 0.08, "kd": 0.12},
    "pitch": {"kp": 1.8, "ki": 0.08, "kd": 0.12},
    "yaw":   {"kp": 2.5, "ki": 0.05, "kd": 0.0},
    "roll_offset":  0.0,
    "pitch_offset": 0.0,
    "base_pressure": 101325.0,
    # Serial port config: [identifier, functionMask, msp_baud_idx, gps_baud_idx,
    #                       telem_baud_idx, periph_baud_idx]
    # Identifiers: USART1=0, USART2=1, USART6=5, USB_VCP=20
    # Functions:   MSP=1, GPS=2, RX_SERIAL=64, BLACKBOX=128
    # Baud index:  AUTO=0, 9600=4, 57600=7, 115200=8, 230400=9, 460800=11
    "ports": [
        [20, 1, 8, 0, 0, 0],   # USB VCP  → MSP (115200)
        [0,  0, 0, 0, 0, 0],   # USART1 X9/X10  → disabled
        [1,  0, 0, 0, 0, 0],   # USART2 Y5/Y6   → disabled
        [5,  0, 0, 0, 0, 0],   # USART6 Y1/Y2   → disabled
    ],
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
