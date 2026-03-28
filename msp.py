"""
MSP (MultiWii Serial Protocol) server — iNAV dialect.

Supports MSP v1 framing for all standard commands and MSP v2 native framing
for iNAV-specific commands (e.g. MSP2_INAV_STATUS = 0x2000).

Wire format (v1):
  Request  : $ M <  <size:u8> <cmd:u8>  [payload]  <xor-checksum:u8>
  Response : $ M >  <size:u8> <cmd:u8>  [payload]  <xor-checksum:u8>

Wire format (v2, used for cmd >= 0x100):
  $ X >  <flag:u8> <cmd:u16-LE> <size:u16-LE>  [payload]  <crc8-dvb-s2:u8>
"""

import struct
import utime

# ---------------------------------------------------------------------------
# MSP command IDs (v1)
# ---------------------------------------------------------------------------
MSP_API_VERSION   = 1
MSP_FC_VARIANT    = 2
MSP_FC_VERSION    = 3
MSP_BOARD_INFO    = 4
MSP_BUILD_INFO    = 5
MSP_INAV_PID      = 6
MSP_SET_INAV_PID  = 7
MSP_NAME          = 10
MSP_FEATURE       = 36
MSP_RX_MAP        = 64
MSP_STATUS        = 101
MSP_RAW_IMU       = 102
MSP_MOTOR         = 104
MSP_ATTITUDE      = 108
MSP_ALTITUDE      = 109
MSP_STATUS_EX     = 150
MSP_SENSOR_STATUS = 151
MSP_EEPROM_WRITE  = 250

# MSP v2 iNAV-specific
MSP2_INAV_STATUS  = 0x2000

# Sensor flag bits (packSensorStatus)
SENSOR_GYRO = 1 << 0
SENSOR_ACC  = 1 << 1
SENSOR_BARO = 1 << 2

# Sensor hardware status values (hardwareSensorStatus_e in iNAV):
#   0 = NONE (not configured), 1 = OK, 2 = UNAVAILABLE, 3 = UNHEALTHY
HW_SENSOR_OK = 1


# ---------------------------------------------------------------------------
# CRC-8/DVB-S2 (used for MSP v2 checksums)
# ---------------------------------------------------------------------------
def _crc8_dvb_s2(buf):
    crc = 0
    for b in buf:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


# ---------------------------------------------------------------------------
# Frame builder helpers
# ---------------------------------------------------------------------------
def _build_v1(cmd, payload=b''):
    size = len(payload)
    cs = size ^ cmd
    for b in payload:
        cs ^= b
    return b'$M>' + bytes([size, cmd]) + payload + bytes([cs & 0xFF])


def _build_v2(cmd, payload=b''):
    size = len(payload)
    header = bytes([0x00]) + struct.pack('<HH', cmd, size)  # flag=0, cmd, size
    crc = _crc8_dvb_s2(header + payload)
    return b'$X>' + header + payload + bytes([crc])


def _pack(*args):
    """Pack list of (fmt_char, value) into bytes."""
    result = bytearray()
    for fmt, val in args:
        if fmt == 'B':
            result.append(val & 0xFF)
        elif fmt == 'b':
            result += struct.pack('<b', val)
        elif fmt == 'H':
            result += struct.pack('<H', val & 0xFFFF)
        elif fmt == 'h':
            result += struct.pack('<h', val)
        elif fmt == 'I':
            result += struct.pack('<I', val & 0xFFFFFFFF)
        elif fmt == 'i':
            result += struct.pack('<i', val)
        elif fmt == 's':
            result += val  # raw bytes
    return bytes(result)


# ---------------------------------------------------------------------------
# MSPServer
# ---------------------------------------------------------------------------
class MSPServer:
    """
    Feed bytes via feed(b). Automatically writes MSP responses to vcp.

    External state is read from the passed-in module references:
      sensors  — provides .roll, .pitch, .yaw, .ax/.ay/.az, .gx/.gy/.gz,
                 .pressure_pa, .temperature_c, .altitude_m
      motors   — provides .outputs list[4] (1000-2000)
      cfg      — dict with 'roll'/'pitch'/'yaw' PID sub-dicts
      save_cfg — callable(cfg) to persist config
    """

    # Parser states
    _IDLE         = 0
    _HEADER_M     = 1  # saw '$'
    _HEADER_X     = 2  # saw '$X' (v2 native)
    _HEADER_ARROW = 3  # saw '$M'
    _SIZE         = 4
    _CMD          = 5
    _PAYLOAD      = 6
    _CHECKSUM     = 7
    # v2 states
    _V2_DIR       = 8   # direction byte '<' after '$X'
    _V2_FLAG      = 9
    _V2_CMD0      = 10
    _V2_CMD1      = 11
    _V2_SIZE0     = 12
    _V2_SIZE1     = 13
    _V2_PAYLOAD   = 14
    _V2_CRC       = 15

    def __init__(self, vcp, sensors, motors, cfg, save_cfg):
        self._vcp = vcp
        self._sensors = sensors
        self._motors = motors
        self._cfg = cfg
        self._save_cfg = save_cfg

        self._state = self._IDLE
        self._size = 0
        self._cmd = 0
        self._payload = bytearray()
        self._cs = 0

        # v2 parse state
        self._v2_flag = 0
        self._v2_cmd = 0
        self._v2_size = 0

        self._cycle_start = utime.ticks_us()
        self._cycle_us = 20000

    def update_cycle(self):
        """Call once per loop to track cycle time for MSP_STATUS."""
        now = utime.ticks_us()
        self._cycle_us = utime.ticks_diff(now, self._cycle_start)
        self._cycle_start = now

    def feed(self, b):
        if isinstance(b, int):
            self._parse(b)
        else:
            for byte in b:
                self._parse(byte)

    # ------------------------------------------------------------------
    # Parser
    # ------------------------------------------------------------------
    def _parse(self, b):
        s = self._state
        if s == self._IDLE:
            if b == ord('$'):
                self._state = self._HEADER_M
        elif s == self._HEADER_M:
            if b == ord('M'):
                self._state = self._HEADER_ARROW
            elif b == ord('X'):
                self._state = self._V2_DIR
            else:
                self._state = self._IDLE
        elif s == self._HEADER_ARROW:
            if b == ord('<'):  # request to FC
                self._state = self._SIZE
            else:
                self._state = self._IDLE
        elif s == self._SIZE:
            self._size = b
            self._cs = b
            self._payload = bytearray()
            self._state = self._CMD
        elif s == self._CMD:
            self._cmd = b
            self._cs ^= b
            self._state = self._PAYLOAD if self._size > 0 else self._CHECKSUM
        elif s == self._PAYLOAD:
            self._payload.append(b)
            self._cs ^= b
            if len(self._payload) == self._size:
                self._state = self._CHECKSUM
        elif s == self._CHECKSUM:
            self._state = self._IDLE
            if (self._cs & 0xFF) == b:
                self._dispatch_v1(self._cmd, bytes(self._payload))
        # --- MSP v2 ---
        elif s == self._V2_DIR:
            # direction byte: '<' = request to FC, '>' = response (ignore)
            if b == ord('<'):
                self._state = self._V2_FLAG
            else:
                self._state = self._IDLE
        elif s == self._V2_FLAG:
            self._v2_flag = b
            self._state = self._V2_CMD0
        elif s == self._V2_CMD0:
            self._v2_cmd = b
            self._state = self._V2_CMD1
        elif s == self._V2_CMD1:
            self._v2_cmd |= b << 8
            self._state = self._V2_SIZE0
        elif s == self._V2_SIZE0:
            self._v2_size = b
            self._state = self._V2_SIZE1
        elif s == self._V2_SIZE1:
            self._v2_size |= b << 8
            self._payload = bytearray()
            self._state = self._V2_PAYLOAD if self._v2_size > 0 else self._V2_CRC
        elif s == self._V2_PAYLOAD:
            self._payload.append(b)
            if len(self._payload) == self._v2_size:
                self._state = self._V2_CRC
        elif s == self._V2_CRC:
            self._state = self._IDLE
            # validate CRC
            header = bytes([self._v2_flag]) + struct.pack('<HH', self._v2_cmd, self._v2_size)
            if _crc8_dvb_s2(header + self._payload) == b:
                self._dispatch_v2(self._v2_cmd, bytes(self._payload))

    # ------------------------------------------------------------------
    # Dispatch
    # ------------------------------------------------------------------
    def _dispatch_v1(self, cmd, payload):
        try:
            resp = self._handle(cmd, payload)
            if resp is not None:
                self._vcp.write(_build_v1(cmd, resp))
        except Exception:
            pass  # never let a handler crash the main loop

    def _dispatch_v2(self, cmd, payload):
        try:
            resp = self._handle(cmd, payload)
            if resp is not None:
                self._vcp.write(_build_v2(cmd, resp))
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Handler dispatch table
    # ------------------------------------------------------------------
    def _handle(self, cmd, payload):
        s = self._sensors
        m = self._motors

        # ---- Handshake ----
        if cmd == MSP_API_VERSION:
            return bytes([0x00, 2, 5])

        if cmd == MSP_FC_VARIANT:
            return b'INAV'

        if cmd == MSP_FC_VERSION:
            return bytes([9, 0, 0])

        if cmd == MSP_BOARD_INFO:
            target = b'PYBD'
            return (b'PYBD'                       # board id (4 bytes)
                    + struct.pack('<H', 0)          # hw revision
                    + bytes([0])                    # OSD type = none
                    + bytes([0x01])                 # comm caps: has VCP
                    + bytes([len(target)])          # target name length
                    + target)                       # target name

        if cmd == MSP_BUILD_INFO:
            # 11-byte date + 8-byte time + 7-byte git hash (all ASCII)
            return b'Mar 29 2026' + b'00:00:00' + b'0000000'

        if cmd == MSP_NAME:
            return b'PyFC'

        # ---- Features / config ----
        if cmd == MSP_FEATURE:
            return struct.pack('<I', 0)

        if cmd == MSP_RX_MAP:
            return bytes([0, 1, 2, 3, 4, 5, 6, 7])

        if cmd == 42:  # MSP_MIXER — QuadX = 3
            return bytes([3])

        if cmd == 113:  # MSP_ACTIVEBOXES — 8-byte boxBitmask, all zero
            return b'\x00' * 8

        # ---- Status ----
        if cmd in (MSP_STATUS, MSP_STATUS_EX):
            sensor_flags = SENSOR_GYRO | SENSOR_ACC | SENSOR_BARO
            p = _pack(
                ('H', self._cycle_us),   # cycle time µs
                ('H', 0),                # i2c errors
                ('H', sensor_flags),     # sensor flags
                ('I', 0),                # active mode box flags (4 bytes)
                ('B', 0),                # profile index
            )
            if cmd == MSP_STATUS_EX:
                p += _pack(
                    ('H', 0),            # system load %
                    ('H', 0),            # arming flags
                    ('B', 0),            # acc calib axis flags
                )
            return p

        if cmd == MSP2_INAV_STATUS:
            # Layout from fc_msp.c:493 (iNAV 9.x):
            #   u16 cycleTime, u16 i2cErrors, u16 sensorFlags,
            #   u16 systemLoad%, u8 (battProfile<<4)|profile,
            #   u32 armingFlags, 8-byte boxBitmask, u8 mixerProfile
            sensor_flags = SENSOR_GYRO | SENSOR_ACC | SENSOR_BARO
            return _pack(
                ('H', self._cycle_us & 0xFFFF),  # cycle time µs (u16)
                ('H', 0),                         # i2c errors
                ('H', sensor_flags),              # sensor flags
                ('H', 0),                         # system load %
                ('B', 0),                         # (battProfile<<4)|profile
                ('I', 0),                         # armingFlags (u32)
                ('s', b'\x00' * 8),               # boxBitmask (60 bits → 8 bytes)
                ('B', 0),                         # mixerProfile
            )

        if cmd == MSP_SENSOR_STATUS:
            return bytes([
                1,            # system healthy
                HW_SENSOR_OK, # gyro
                HW_SENSOR_OK, # acc
                0,            # mag (none)
                HW_SENSOR_OK, # baro
                0,            # gps
                0,            # rangefinder
                0,            # pitot
                0,            # opflow
            ])

        # ---- Telemetry ----
        if cmd == MSP_RAW_IMU:
            # acc * 512 (in G), gyro in dps, mag = 0
            def i16(v):
                iv = int(v)
                return struct.pack('<h', max(-32768, min(32767, iv)))
            p = bytearray()
            for v in (s.ax, s.ay, s.az):
                p += i16(v * 512)
            for v in (s.gx, s.gy, s.gz):
                p += i16(v)
            p += b'\x00\x00\x00\x00\x00\x00'  # mag = 0
            return bytes(p)

        if cmd == MSP_MOTOR:
            p = bytearray()
            for i in range(8):
                val = m.outputs[i] if i < 4 else 0
                p += struct.pack('<H', val)
            return bytes(p)

        if cmd == MSP_ATTITUDE:
            roll_dd  = int(s.roll  * 10)
            pitch_dd = int(s.pitch * 10)
            yaw_deg  = int(s.yaw) % 360
            return _pack(('h', roll_dd), ('h', pitch_dd), ('h', yaw_deg))

        if cmd == MSP_ALTITUDE:
            alt_cm   = int(s.altitude_m * 100)
            baro_cm  = alt_cm
            vario    = 0
            return _pack(('i', alt_cm), ('h', vario), ('i', baro_cm))

        # ---- PID get/set (iNAV variant, cmd 6/7) ----
        if cmd == MSP_INAV_PID:
            cfg = self._cfg
            def pid_bytes(axis):
                return _pack(
                    ('H', int(cfg[axis]['kp'] * 1000)),
                    ('H', int(cfg[axis]['ki'] * 1000)),
                    ('H', int(cfg[axis]['kd'] * 1000)),
                )
            return pid_bytes('roll') + pid_bytes('pitch') + pid_bytes('yaw')

        if cmd == MSP_SET_INAV_PID:
            if len(payload) >= 18:
                axes = ('roll', 'pitch', 'yaw')
                for i, axis in enumerate(axes):
                    off = i * 6
                    kp = struct.unpack_from('<H', payload, off)[0] / 1000.0
                    ki = struct.unpack_from('<H', payload, off + 2)[0] / 1000.0
                    kd = struct.unpack_from('<H', payload, off + 4)[0] / 1000.0
                    self._cfg[axis] = {'kp': kp, 'ki': ki, 'kd': kd}
            return b''  # ACK with empty payload

        if cmd == MSP_EEPROM_WRITE:
            self._save_cfg(self._cfg)
            return b''  # ACK

        # ---- Unique device ID ----
        if cmd == 160:  # MSP_UID — 3×u32 = 12 bytes
            return _pack(('I', 0xDEAD0001), ('I', 0xDEAD0002), ('I', 0xDEAD0003))

        # ---- iNAV v2 telemetry (required for landing page) ----
        if cmd == 0x2002:  # MSP2_INAV_ANALOG — 24 bytes
            # u8 battFlags, u16 voltage(cV), u16 amperage(10mA),
            # u32 power(mW), u32 mAh, u32 mWh, u32 remaining, u8 %, u16 rssi
            return _pack(
                ('B', 0),   # battery flags (0 = no battery)
                ('H', 0),   # voltage
                ('H', 0),   # amperage
                ('I', 0),   # power
                ('I', 0),   # mAh drawn
                ('I', 0),   # mWh drawn
                ('I', 0),   # remaining capacity
                ('B', 0),   # battery %
                ('H', 0),   # RSSI
            )

        if cmd == 0x2003:  # MSP2_INAV_MISC — 41 bytes (no GPS, no ADC path)
            return _pack(
                ('H', 1500), ('H', 0), ('H', 2000), ('H', 1000), ('H', 1000),  # throttle settings
                ('B', 0), ('B', 0), ('B', 0),  # gps type/baud/sbas
                ('B', 0),   # rssi channel
                ('H', 0),   # mag declination
                # without ADC: voltage scale, source, cells, 4×cell volts
                ('H', 0), ('B', 0), ('B', 0), ('H', 0), ('H', 0), ('H', 0), ('H', 0),
                # capacity: value, warning, critical (u32 each), unit (u8)
                ('I', 0), ('I', 0), ('I', 0), ('B', 0),
            )

        if cmd == 0x2005:  # MSP2_INAV_BATTERY_CONFIG — 29 bytes
            # without ADC (12 bytes) + always (17 bytes)
            return _pack(
                ('H', 0), ('B', 0), ('B', 0), ('H', 0), ('H', 0), ('H', 0), ('H', 0),
                ('H', 0), ('H', 0),  # current offset, scale
                ('I', 0), ('I', 0), ('I', 0), ('B', 0),  # capacity + unit
            )

        if cmd == 0x203A:  # MSP2_INAV_MISC2 — 10 bytes
            # u32 onTime(s), u32 flightTime(s), u8 throttle%, u8 autoThrottle
            import utime as _t
            return _pack(
                ('I', _t.ticks_ms() // 1000),
                ('I', 0),
                ('B', 0),
                ('B', 0),
            )

        if cmd == 0x2030:  # MSP2_PID — 11 axes × 4 bytes (P,I,D,FF) = 44 bytes
            cfg = self._cfg
            # Map our 3 axes to iNAV's PID_ITEM_COUNT=11; rest zero
            # Order: ROLL, PITCH, YAW, POS_Z, POS_XY, VEL_XY, SURFACE, LEVEL, HEADING, VEL_Z, POS_HEADING
            axis_map = {0: 'roll', 1: 'pitch', 2: 'yaw'}
            p = bytearray()
            for i in range(11):
                if i in axis_map:
                    a = cfg[axis_map[i]]
                    p += bytes([
                        min(255, int(a['kp'] * 10)),
                        min(255, int(a['ki'] * 10)),
                        min(255, int(a['kd'] * 10)),
                        0,  # FF
                    ])
                else:
                    p += b'\x00\x00\x00\x00'
            return bytes(p)

        if cmd == 0x2031:  # MSP2_SET_PID — update PID and ACK
            # payload: 11 axes × 4 bytes
            if len(payload) >= 12:
                axes = {0: 'roll', 1: 'pitch', 2: 'yaw'}
                for i, key in axes.items():
                    off = i * 4
                    self._cfg[key] = {
                        'kp': payload[off]   / 10.0,
                        'ki': payload[off+1] / 10.0,
                        'kd': payload[off+2] / 10.0,
                    }
            return b''

        # ---- Fixed-struct config commands (wrong size = JS parser exception) ----

        if cmd == 44:  # MSP_RX_CONFIG — 24 bytes
            # u8 provider, u16 maxcheck, u16 midrc, u16 mincheck,
            # u8 spektrum_bind, u16 rx_min_usec, u16 rx_max_usec,
            # u8 rcInterp, u8 rcInterpInterval, u16 airModeThresh,
            # u8 0, u32 0, u8 0, u8 fpvCamAngle, u8 receiverType
            return _pack(
                ('B', 0), ('H', 1900), ('H', 1500), ('H', 1100),
                ('B', 0), ('H', 885),  ('H', 2115),
                ('B', 0), ('B', 19),   ('H', 0),
                ('B', 0), ('I', 0),    ('B', 0), ('B', 0), ('B', 0),
            )

        if cmd == 75:  # MSP_FAILSAFE_CONFIG — 20 bytes
            return _pack(
                ('B', 5),   # failsafe_delay
                ('B', 200), # failsafe_off_delay
                ('H', 1000),# failsafe_throttle
                ('B', 0),   # was kill_switch
                ('H', 0),   # failsafe_throttle_low_delay
                ('B', 0),   # failsafe_procedure (DROP)
                ('B', 5),   # failsafe_recovery_delay
                ('H', 0), ('H', 0), ('H', 0),  # fw angles
                ('H', 50),  # stick_motion_threshold
                ('H', 0),   # min_distance
                ('B', 0),   # min_distance_procedure
            )

        if cmd == 38:  # MSP_BOARD_ALIGNMENT — 6 bytes (3×i16 decidegrees)
            return _pack(('h', 0), ('h', 0), ('h', 0))

        if cmd == 73:  # MSP_LOOP_TIME — u16
            return _pack(('H', 1000))

        if cmd == 50:  # MSP_RSSI_CONFIG — u8
            return bytes([0])

        if cmd == 0x2010:  # MSP2_INAV_MIXER — 9 bytes
            # u8 motorDirInverted, u8 0, u8 motorstopOnLow, u8 platformType,
            # u8 hasFlaps, u16 appliedMixerPreset, u8 maxMotors, u8 maxServos
            return _pack(
                ('B', 0), ('B', 0), ('B', 0), ('B', 0),  # platformType=0 = MULTIROTOR
                ('B', 0), ('H', -1), ('B', 8), ('B', 8),
            )

        if cmd == 0x200A:  # MSP2_INAV_OUTPUT_MAPPING — 1 byte per output
            # 4 motors: TIM_USE_MOTOR = 0x01
            return bytes([1, 1, 1, 1])

        if cmd == 0x210D:  # MSP2_INAV_OUTPUT_MAPPING_EXT2 — 6 bytes per output
            # Each: u8 timerId, u32 usageFlags, u8 label
            # 4 motors: timerId=0..3, flags=TIM_USE_MOTOR=1, label=0
            p = bytearray()
            for i in range(4):
                p += bytes([i]) + struct.pack('<I', 1) + bytes([0])
            return bytes(p)

        # MSP_V2_FRAME (255) — v2-over-v1 wrapper: unwrap and re-dispatch
        if cmd == 255 and len(payload) >= 6:
            v2_flag = payload[0]
            v2_cmd  = payload[1] | (payload[2] << 8)
            v2_size = payload[3] | (payload[4] << 8)
            v2_data = payload[5:5 + v2_size]
            v2_crc  = payload[5 + v2_size] if len(payload) > 5 + v2_size else 0
            hdr = bytes([v2_flag]) + struct.pack('<HH', v2_cmd, v2_size)
            if _crc8_dvb_s2(hdr + v2_data) == v2_crc:
                resp = self._handle(v2_cmd, bytes(v2_data))
                if resp is not None:
                    self._vcp.write(_build_v2(v2_cmd, resp))
            return None  # don't also send a v1 response

        # Unknown command — send empty ACK so Configurator doesn't hang
        return b''
