"""mick_chassis_protocol.py

Mirrors mick_chassis_protocol.h / mick_chassis_protocol.cpp:

  class MickRobotCHASSIS:
      MickRobotCHASSIS(mode, chassis_type, dev, baud, udp_ip, udp_port)
      readData()                       -> bool
      analy_uart_recive_data(raw)      -> bool
      send_speed_to_chassis(x, y, w)
      send_rpm_to_chassis(w1, w2, w3, w4)
      send_speed_to_X4chassis(x, y, w)
      send_speed_to_Ackerchassis(x, w)
      send_speed_to_4WS4WDchassis(x, y, w)
      send_rpm_to_4WS4WDchassis(vw)
      clear_odometry_chassis()
      read_bytes_from_udp_or_serial() -> bytes
      send_bytes_to_udp_or_serial(data)
      calculate_chassisDiffX4_position_for_odometry()

Frame format (same as C++ implementation):
    [0xAE] [0xEA] [len] [cmd] [payload...] [check] [0xFF] [0xEF] [0xFE]
where `len` is the byte from index 2 onwards (without the 0xAE 0xEA header
and 0xEF 0xFE tail), and `check` is the sum of all bytes between len..check
mod 256 (also accepts 0xFF as a wildcard).
"""

import math
import time
import socket

from .mick_chassis_msg import ChassisMeasure

# ---------------------------------------------------------------------------
# Wheel geometry constants (same names / values as mick_chassis_protocol.h)
# ---------------------------------------------------------------------------
WHEEL_RATIO = 19.0
WHEEL_K     = 0.355
WHEEL_L     = 0.4
WHEEL_D     = 0.17
WHEEL_PI    = 3.141693


# ---------------------------------------------------------------------------
# Byte helpers (replaces INT32Data / Int16Data union trick in C++)
# ---------------------------------------------------------------------------
def int16_from_be(high: int, low: int) -> int:
    """Big-endian int16 from two unsigned bytes."""
    val = (high << 8) | low
    if val >= 0x8000:
        val -= 0x10000
    return val


def int32_from_be(b3: int, b2: int, b1: int, b0: int) -> int:
    """Big-endian int32 from four unsigned bytes."""
    val = (b3 << 24) | (b2 << 16) | (b1 << 8) | b0
    if val >= 0x80000000:
        val -= 0x100000000
    return val


def _build_frame(cmd: int, payload: bytes) -> bytes:
    """Build a complete frame: header + cmd + payload + check + tail.
    Mirrors the C++ pattern where:
        data_tem[2] = counter - 2   (length byte)
        data_tem[len-3] = 0xFF     (checksum byte, wildcard)
    """
    body = bytearray()
    body.append(0xAE)
    body.append(0xEA)
    body.append(0x0B)              # placeholder for length, fixed up later
    body.append(cmd)
    body.extend(payload)
    # checksum (sum of bytes from `len` position through end of payload)
    check = sum(body[2:]) & 0xFF
    body.append(0xFF)              # wildcard checksum
    body[2] = len(body) - 2        # fix up length
    body.append(0xEF)
    body.append(0xFE)
    return bytes(body)


# ---------------------------------------------------------------------------
# MickRobotCHASSIS class
# ---------------------------------------------------------------------------
class MickRobotCHASSIS:
    def __init__(self, mode: int, chassis_type: int,
                 dev: str, baud: int,
                 udp_serv_ip: str, udp_serv_port: int):
        """
        mode         : 0 → serial, 1 → UDP
        chassis_type : 0:X4  1:M4  2:Ackermann  3:4WS4WD
        dev          : serial device path, e.g. /dev/ttyUSB0
        baud         : serial baud rate
        udp_serv_ip  : UDP server IP
        udp_serv_port: UDP server port
        """
        self.communication_mode_ = mode
        self.chassis_measurements = ChassisMeasure()
        self.chassis_measurements.chassis_type = chassis_type

        self.dev_  = dev
        self.baud_ = baud
        self.udp_serv_ip_   = udp_serv_ip if udp_serv_ip else "192.168.1.30"
        self.udp_port_      = udp_serv_port if udp_serv_port else 1231

        # Odometry integration state
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_w = 0.0
        self.motor_init_flag = 0
        self.last_time = time.monotonic()

        # Transport handles
        self._serial = None
        self._udp_sock = None

        if self.communication_mode_ == 0:
            self._open_serial()
        else:
            self._open_udp()

    # -- transport open -----------------------------------------------------
    def _open_serial(self):
        try:
            import serial
            self._serial = serial.Serial(
                port=self.dev_,
                baudrate=self.baud_,
                timeout=0.01,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
            )
        except Exception as e:
            print(f"Failed to open serial port {self.dev_}: {e}")

    def _open_udp(self):
        self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._udp_sock.setblocking(False)
        try:
            self._udp_sock.bind(('', self.udp_port_))
        except OSError as e:
            print(f"UDP bind failed: {e}")
            return
        print(f"UDP server started on port {self.udp_port_}")

    # -- transport close ----------------------------------------------------
    def close(self):
        if self._serial and self._serial.is_open:
            self._serial.close()
        if self._udp_sock:
            self._udp_sock.close()

    # -- low-level read / write --------------------------------------------
    def read_bytes_from_udp_or_serial(self) -> bytes:
        if self.communication_mode_ == 0:
            if self._serial and self._serial.in_waiting:
                return self._serial.read(self._serial.in_waiting)
            return b''
        else:
            try:
                data, _ = self._udp_sock.recvfrom(1024)
                return data
            except BlockingIOError:
                return b''
            except Exception:
                return b''

    def send_bytes_to_udp_or_serial(self, data: bytes):
        if self.communication_mode_ == 0:
            if self._serial and self._serial.is_open:
                self._serial.write(data)
        else:
            if self._udp_sock:
                self._udp_sock.sendto(data, (self.udp_serv_ip_, self.udp_port_))

    # -- public API: read / send speed -------------------------------------
    def readData(self) -> bool:
        raw = self.read_bytes_from_udp_or_serial()
        if len(raw) <= 5:
            return False
        if self.analy_uart_recive_data(raw):
            ct = self.chassis_measurements.chassis_type
            if ct == 0 or ct == 1:
                self.calculate_chassisDiffX4_position_for_odometry()
            return True
        return False

    def send_speed_to_chassis(self, speed_x: float, speed_y: float, speed_w: float):
        ct = self.chassis_measurements.chassis_type
        if ct == 0:                                          # mickX4-v3 diff drive
            self.send_speed_to_X4chassis(speed_x, speed_y, speed_w)
        elif ct == 1:                                        # Mecanum
            v1 =  speed_x - speed_y - WHEEL_K * speed_w
            v2 =  speed_x + speed_y - WHEEL_K * speed_w
            v3 = -(speed_x - speed_y + WHEEL_K * speed_w)
            v4 = -(speed_x + speed_y + WHEEL_K * speed_w)
            v1 = v1 / (WHEEL_D * WHEEL_PI) * 60
            v2 = v2 / (WHEEL_D * WHEEL_PI) * 60
            v3 = v3 / (WHEEL_D * WHEEL_PI) * 60
            v4 = v4 / (WHEEL_D * WHEEL_PI) * 60
            self.send_rpm_to_chassis(int(v1), int(v2), int(v3), int(v4))
        elif ct == 2:                                        # Ackermann
            self.send_speed_to_Ackerchassis(speed_x, speed_w)
        elif ct == 3:                                        # 4WS4WD
            self.send_speed_to_4WS4WDchassis(speed_x, speed_y, speed_w)

    # -- send frames --------------------------------------------------------
    def send_speed_to_X4chassis(self, x: float, y: float, w: float):
        speed_offset = 10
        payload = bytearray()
        payload.append(int(((x + speed_offset) * 100)) // 256)
        payload.append(int(((x + speed_offset) * 100)) & 0xFF)
        payload.append(int(((y + speed_offset) * 100)) // 256)
        payload.append(int(((y + speed_offset) * 100)) & 0xFF)
        payload.append(int(((w + speed_offset) * 100)) // 256)
        payload.append(int(((w + speed_offset) * 100)) & 0xFF)
        payload.append(0x00)
        payload.append(0x00)
        self.send_bytes_to_udp_or_serial(_build_frame(0xF3, bytes(payload)))

    def send_rpm_to_chassis(self, w1: int, w2: int, w3: int, w4: int):
        speed_offset = 10000
        payload = bytearray()
        for spd in (w1, w2, w3, w4):
            v = spd + speed_offset
            payload.append(v // 256)
            payload.append(v & 0xFF)
        self.send_bytes_to_udp_or_serial(_build_frame(0xF1, bytes(payload)))

    def send_speed_to_Ackerchassis(self, x: float, w: float):
        speed_offset = 10
        payload = bytearray()
        payload.append(int(((x + speed_offset) * 100)) // 256)
        payload.append(int(((x + speed_offset) * 100)) & 0xFF)
        payload.append(0x00)
        payload.append(0x00)
        payload.append(int(((w + speed_offset) * 100)) // 256)
        payload.append(int(((w + speed_offset) * 100)) & 0xFF)
        self.send_bytes_to_udp_or_serial(_build_frame(0xF4, bytes(payload)))

    def send_speed_to_4WS4WDchassis(self, x: float, y: float, w: float):
        speed_offset = 10
        payload = bytearray()
        payload.append(int(((x + speed_offset) * 100)) // 256)
        payload.append(int(((x + speed_offset) * 100)) & 0xFF)
        payload.append(int(((y + speed_offset) * 100)) // 256)
        payload.append(int(((y + speed_offset) * 100)) & 0xFF)
        payload.append(int(((w + speed_offset) * 100)) // 256)
        payload.append(int(((w + speed_offset) * 100)) & 0xFF)
        self.send_bytes_to_udp_or_serial(_build_frame(0xF2, bytes(payload)))

    def send_rpm_to_4WS4WDchassis(self, vw):
        if len(vw) < 12:
            print("For 4ws4wd chassis, the length of vw < 12")
            return
        speed_offset = 10000
        theta_offset = 360
        payload = bytearray()
        i = 0
        for _ in range(6):
            t = (vw[i] + theta_offset) * 100
            payload.append(int(t) // 256)
            payload.append(int(t) & 0xFF)
            i += 1
            s = int(vw[i] + speed_offset)
            payload.append(s // 256)
            payload.append(s & 0xFF)
            i += 1
        self.send_bytes_to_udp_or_serial(_build_frame(0xFA, bytes(payload)))

    def clear_odometry_chassis(self):
        payload = bytearray()
        payload.append(0x01)            # command: clear odometry
        payload.extend([0x00] * 7)
        self.send_bytes_to_udp_or_serial(_build_frame(0xE1, bytes(payload)))

    # -- frame parser (mirrors analy_uart_recive_data) ---------------------
    def analy_uart_recive_data(self, raw: bytes) -> bool:
        if len(raw) < 5:
            return False

        # Extract complete frames 0xAE 0xEA ... 0xEF 0xFE
        frames = []
        rec_flag = False
        frame_buf = bytearray()
        tem_last = 0
        for b in raw:
            tem_curr = b
            if tem_last == 0xAE and tem_curr == 0xEA and not rec_flag:
                rec_flag = True
                frame_buf = bytearray([tem_last, tem_curr])
            elif rec_flag:
                frame_buf.append(b)
                if tem_last == 0xEF and tem_curr == 0xFE:
                    frames.append(bytes(frame_buf))
                    rec_flag = False
            tem_last = tem_curr

        if not frames:
            return False

        for frame in frames:
            if len(frame) < 6:
                continue
            if frame[0] != 0xAE or frame[1] != 0xEA:
                continue
            flen = frame[2] + 4
            if flen != len(frame):
                continue
            if frame[flen - 2] != 0xEF or frame[flen - 1] != 0xFE:
                continue

            # checksum: sum of bytes between [2, flen-3), accepted if matches
            # or if check byte is the wildcard 0xFF (same as C++).
            check_sum = sum(frame[2:flen - 3]) & 0xFF
            if frame[flen - 3] != check_sum and frame[flen - 3] != 0xFF:
                print(f"check_sum error: 0x{check_sum:02X}, frame: 0x{frame[flen - 3]:02X}")
                continue

            cmd = frame[3]
            if   cmd == 0x07: self._parse_motor_data(frame)
            elif cmd == 0xA7: self._parse_odom_data(frame)
            elif cmd == 0xA0: self._parse_imu_data(frame)
            elif cmd == 0xA3: self._parse_rc_data(frame)
        return True

    # -- sub-parsers --------------------------------------------------------
    def _parse_motor_data(self, frame: bytes):
        i = 4
        for j in range(4):
            spd  = int32_from_be(frame[i],   frame[i+1], frame[i+2], frame[i+3]); i += 4
            rcnt = int32_from_be(frame[i],   frame[i+1], frame[i+2], frame[i+3]); i += 4
            ang  = int32_from_be(frame[i],   frame[i+1], frame[i+2], frame[i+3]); i += 4
            m = self.chassis_measurements.moto_measurements[j]
            m.speed_rpm = spd
            m.round_cnt = rcnt
            m.angle     = ang
            m.available = 1

    def _parse_odom_data(self, frame: bytes):
        i = 4
        vx = int16_from_be(frame[i], frame[i+1]) / 1000.0; i += 2
        vy = int16_from_be(frame[i], frame[i+1]) / 1000.0; i += 2
        wz = int16_from_be(frame[i], frame[i+1]) / 1000.0; i += 2
        om = self.chassis_measurements.odom_measurements
        om.vx, om.vy, om.wz = vx, vy, wz
        om.available = 1

    def _parse_imu_data(self, frame: bytes):
        imu = self.chassis_measurements.imu_measurements
        i = 4
        imu.ax = int16_from_be(frame[i], frame[i+1]); i += 2
        imu.ay = int16_from_be(frame[i], frame[i+1]); i += 2
        imu.az = int16_from_be(frame[i], frame[i+1]); i += 2
        imu.gx = int16_from_be(frame[i], frame[i+1]); i += 2
        imu.gy = int16_from_be(frame[i], frame[i+1]); i += 2
        imu.gz = int16_from_be(frame[i], frame[i+1]); i += 2
        imu.mx = int16_from_be(frame[i], frame[i+1]); i += 2
        imu.my = int16_from_be(frame[i], frame[i+1]); i += 2
        imu.mz = int16_from_be(frame[i], frame[i+1]); i += 2
        imu.pitch = int16_from_be(frame[i], frame[i+1]) / 100.0; i += 2
        imu.roll  = int16_from_be(frame[i], frame[i+1]) / 100.0; i += 2
        imu.yaw   = int16_from_be(frame[i], frame[i+1]) / 100.0; i += 2
        imu.available = 1

    def _parse_rc_data(self, frame: bytes):
        rc = self.chassis_measurements.rc
        i = 4
        rc.ch1 = (frame[i] << 8) | frame[i+1]; i += 2
        rc.ch2 = (frame[i] << 8) | frame[i+1]; i += 2
        rc.ch3 = (frame[i] << 8) | frame[i+1]; i += 2
        rc.ch4 = (frame[i] << 8) | frame[i+1]; i += 2
        rc.sw1 = frame[i]; i += 1
        rc.sw2 = frame[i]; i += 1
        rc.sw3 = frame[i]; i += 1
        rc.sw4 = frame[i]; i += 1
        rc.type   = frame[i]; i += 1
        rc.status = frame[i]; i += 1
        rc.update = 1
        if 0 <= rc.ch1 <= 2700:
            rc.available = 1

    # -- odometry integration ----------------------------------------------
    def calculate_chassisDiffX4_position_for_odometry(self):
        if (self.motor_init_flag == 0 and
                self.chassis_measurements.moto_measurements[0].counter == 0):
            self.position_x = 0.0
            self.position_y = 0.0
            self.position_w = 0.0
            self.last_time = time.monotonic()
            self.motor_init_flag += 1
            return

        om = self.chassis_measurements.odom_measurements
        if not om.available:
            return
        linear_x = om.vx
        linear_y = 0.0
        linear_w = om.wz

        # dead-zone (same as C++)
        if abs(linear_x) < 0.01:   linear_x = 0.0
        if abs(linear_w) < 0.001:  linear_w = 0.0

        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now
        if dt > 1.0:
            dt = 0.0

        self.position_x += math.cos(self.position_w) * linear_x * dt
        self.position_y += math.sin(self.position_w) * linear_x * dt
        self.position_w += linear_w * dt

        if self.position_w > 2 * WHEEL_PI:
            self.position_w -= 2 * WHEEL_PI
        elif self.position_w < -2 * WHEEL_PI:
            self.position_w += 2 * WHEEL_PI

        om.x   = self.position_x
        om.y   = self.position_y
        om.yaw = self.position_w
