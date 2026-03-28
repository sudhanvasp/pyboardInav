"""
crsf.py — ExpressLRS / CRSF protocol receiver.

Connect ELRS receiver CRSF TX → pyboard Y2 (UART 6 RX).
CRSF TX from pyboard Y1 → ELRS RX for telemetry (optional).

Baud: 420000, 8N1.

Usage:
    from machine import UART
    import crsf
    uart = UART(6, 420000)
    rx = crsf.CRSFReceiver()
    # in loop:
    if uart.any():
        rx.feed(uart.read())
    throttle = rx.channels[2]   # CH3, 1000-2000 µs
"""

# Frame sync bytes
_SYNC_FC   = 0xC8   # flight controller address (ELRS → FC)
_SYNC_BCAST = 0x00  # broadcast

# Frame types
RC_CHANNELS_PACKED = 0x16   # 22-byte payload, 16× 11-bit channels

# Raw channel range → µs
_RAW_MIN = 172
_RAW_MAX = 1811
_RAW_RANGE = _RAW_MAX - _RAW_MIN  # 1639


def _crc8(data):
    """CRC-8/DVB-S2, covers frame_type + payload."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _raw_to_us(raw):
    """Map CRSF raw value (172-1811) to µs (1000-2000)."""
    return max(1000, min(2000, (raw - _RAW_MIN) * 1000 // _RAW_RANGE + 1000))


class CRSFReceiver:
    """
    Stateful CRSF byte-stream parser.

    channels[0..15] — RC channel values in µs (1000-2000).
      CH1 (idx 0): Roll
      CH2 (idx 1): Pitch
      CH3 (idx 2): Throttle  (1000 when stick at bottom)
      CH4 (idx 3): Yaw
      CH5 (idx 4): Arm switch  (>1700 = armed)

    armed   — True when CH5 > 1700
    healthy — True while frames arrive; False after ~200 ms silence
    """

    _IDLE    = 0
    _LEN     = 1
    _TYPE    = 2
    _PAYLOAD = 3
    _CRC     = 4

    def __init__(self):
        self.channels = [1500] * 16
        self.channels[2] = 1000   # throttle low by default
        self.armed   = False
        self.healthy = False

        self._state       = self._IDLE
        self._payload_len = 0
        self._type        = 0
        self._payload     = bytearray()
        self._last_frame_ms = 0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def feed(self, data):
        """Feed raw bytes from UART read()."""
        if data:
            for b in data:
                self._parse(b)

    def check_timeout(self, now_ms, timeout_ms=200):
        """Call each loop. Sets healthy=False if no frame received recently."""
        if self._last_frame_ms and (now_ms - self._last_frame_ms) > timeout_ms:
            self.healthy = False
            self.armed   = False

    # ------------------------------------------------------------------
    # Parser state machine
    # ------------------------------------------------------------------
    def _parse(self, b):
        s = self._state

        if s == self._IDLE:
            # Accept FC address or broadcast as frame start
            if b == _SYNC_FC or b == _SYNC_BCAST:
                self._state = self._LEN

        elif s == self._LEN:
            # Length = type(1) + payload(N) + crc(1)
            # Valid CRSF frames: 4..64 bytes total
            if 2 <= b <= 62:
                self._payload_len = b - 2   # bytes between TYPE and CRC
                self._state = self._TYPE
            else:
                self._state = self._IDLE

        elif s == self._TYPE:
            self._type = b
            self._payload = bytearray()
            if self._payload_len > 0:
                self._state = self._PAYLOAD
            else:
                self._state = self._CRC

        elif s == self._PAYLOAD:
            self._payload.append(b)
            if len(self._payload) == self._payload_len:
                self._state = self._CRC

        elif s == self._CRC:
            self._state = self._IDLE
            expected = _crc8(bytes([self._type]) + self._payload)
            if b == expected:
                self._dispatch(self._type, bytes(self._payload))

    # ------------------------------------------------------------------
    # Frame handlers
    # ------------------------------------------------------------------
    def _dispatch(self, frame_type, payload):
        import utime
        self._last_frame_ms = utime.ticks_ms()
        self.healthy = True

        if frame_type == RC_CHANNELS_PACKED and len(payload) == 22:
            self._unpack_channels(payload)
            # Update arm state from CH5 (index 4)
            self.armed = self.channels[4] > 1700

    def _unpack_channels(self, data):
        """Unpack 16 × 11-bit channels from 22-byte payload."""
        buf  = 0
        bits = 0
        idx  = 0
        for byte in data:
            buf  |= byte << bits
            bits += 8
            while bits >= 11 and idx < 16:
                self.channels[idx] = _raw_to_us(buf & 0x7FF)
                buf  >>= 11
                bits  -= 11
                idx   += 1
