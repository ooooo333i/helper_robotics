class RobotParameters:
    """Shared robot and MD200T motor driver parameters."""

    def __init__(self):
        self.WHEEL_RADIUS = 0.065
        self.TRACK_WIDTH = 0.400
        self.GEAR_RATIO = 49.0

        self.MAX_RPM = 4000
        self.MAX_LINEAR_VEL = 1.0
        self.MAX_ANGULAR_VEL = 1.5

        self.ODOM_PUBLISH_RATE = 20.0

        self.SERIAL_PORT = '/dev/ttyUSB0'
        self.BAUD_RATE = 57600
        self.DRIVER_ID = 1

    def print_robot_status(self):
        print(
            '[Helper Robotics] Config Loaded: '
            f'R={self.WHEEL_RADIUS}m, '
            f'L={self.TRACK_WIDTH}m, '
            f'Gear={self.GEAR_RATIO}, '
            f'Baudrate={self.BAUD_RATE}bps'
        )
