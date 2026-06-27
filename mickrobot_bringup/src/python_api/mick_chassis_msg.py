"""mick_chassis_msg.py

Data structures mirroring mick_chassis_msg.h:
  - INT32Data / Int16Data  (union helpers → replaced by byte-level helpers)
  - moto_measure_t   → MotoMeasure
  - imu_measure_t    → ImuMeasure
  - rc_info_t        → RcInfo
  - odom_measure_t   → OdomMeasure
  - chassis_measure_t → ChassisMeasure
"""


class MotoMeasure:
    """Mirrors moto_measure_t."""
    __slots__ = ('available', 'counter', 'angle', 'last_angle',
                 'speed_rpm', 'given_current', 'offset_angle',
                 'round_cnt', 'total_angle', 'Temp')

    def __init__(self):
        for s in self.__slots__:
            setattr(self, s, 0)


class ImuMeasure:
    """Mirrors imu_measure_t."""
    __slots__ = ('available', 'counter',
                 'ax', 'ay', 'az', 'gx', 'gy', 'gz',
                 'mx', 'my', 'mz',
                 'pitch', 'roll', 'yaw',
                 'pitch_rad', 'roll_rad', 'yaw_rad',
                 'qw', 'qx', 'qy', 'qz')

    def __init__(self):
        for s in self.__slots__:
            setattr(self, s, 0.0)


class RcInfo:
    """Mirrors rc_info_t."""
    __slots__ = ('ch1', 'ch2', 'ch3', 'ch4',
                 'ch5', 'ch6', 'ch7', 'ch8', 'ch9', 'ch10',
                 'ch11', 'ch12', 'ch13', 'ch14', 'ch15', 'ch16',
                 'ch17', 'ch18',
                 'sw1', 'sw2', 'sw3', 'sw4',
                 'ch1_offset', 'ch2_offset', 'ch3_offset', 'ch4_offset',
                 'type', 'status', 'update', 'available')

    def __init__(self):
        for s in self.__slots__:
            setattr(self, s, 0)


class OdomMeasure:
    """Mirrors odom_measure_t."""
    __slots__ = ('available', 'x', 'y', 'z',
                 'vx', 'vy', 'vz',
                 'wx', 'wy', 'wz',
                 'roll', 'pitch', 'yaw',
                 'qx', 'qy', 'qz', 'qw')

    def __init__(self):
        for s in self.__slots__:
            setattr(self, s, 0.0)


class ChassisMeasure:
    """Mirrors chassis_measure_t."""
    def __init__(self):
        self.chassis_type = 0          # 0:X4  1:M4  2:Ackermann  3:4WS4WD
        self.chassis_motor_type = 0    # 0:M3508  1:安普斯电机
        self.odom_measurements = OdomMeasure()
        self.moto_measurements = [MotoMeasure() for _ in range(4)]
        self.moto_rmd_measurements = [MotoMeasure() for _ in range(4)]
        self.rc = RcInfo()
        self.imu_measurements = ImuMeasure()
        self.ultrasonic = []
