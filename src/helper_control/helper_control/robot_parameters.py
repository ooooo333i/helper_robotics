class RobotParameters:
    """Shared robot and MD200T motor driver parameters."""

    def __init__(self):
        # Kinematics
        self.WHEEL_RADIUS = 0.065
        self.TRACK_WIDTH = 0.190
        self.GEAR_RATIO = 49.0

        # Motion limits
        self.MAX_RPM = 4000
        self.MAX_LINEAR_VEL = 1.0
        self.MAX_ANGULAR_VEL = 1.5

        # Motor direction calibration.
        # Confirmed on the real robot:
        # left +RPM = backward, left -RPM = forward
        # right +RPM = backward, right -RPM = forward
        self.LEFT_FORWARD_SIGN = -1
        self.RIGHT_FORWARD_SIGN = -1
        self.SWAP_MOTORS = False
        self.LEFT_RPM_SCALE = 1.0
        self.RIGHT_RPM_SCALE = 1.0

        # Speed ramp and stop behavior
        self.CONTROL_PERIOD = 0.05
        self.CMD_TIMEOUT = 0.5
        self.COM_WATCH_DELAY = 5
        self.USE_CMD_BRAKE_ON_STOP = True
        self.BRAKE_DELAY_SEC = 1.0
        self.ACCEL_RPM_PER_SEC = 3000.0
        self.DECEL_RPM_PER_SEC = 6000.0
        self.STOP_RPM_PER_SEC = 20000.0

        # Motor driver
        self.SERIAL_PORT = (
            '/dev/serial/by-id/'
            'usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0'
        )
        self.BAUD_RATE = 57600
        self.DRIVER_ID = 1

        # Odometry placeholder
        self.ODOM_PUBLISH_RATE = 20.0

    def print_robot_status(self):
        print(
            '[Helper Robotics] Config Loaded: '
            f'R={self.WHEEL_RADIUS}m, '
            f'L={self.TRACK_WIDTH}m, '
            f'Gear={self.GEAR_RATIO}, '
            f'Baudrate={self.BAUD_RATE}bps, '
            f'LeftSign={self.LEFT_FORWARD_SIGN}, '
            f'RightSign={self.RIGHT_FORWARD_SIGN}'
        )
