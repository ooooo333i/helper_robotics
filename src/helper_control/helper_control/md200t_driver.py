import struct
import threading
import time

try:
    import serial
except ImportError:
    serial = None


class MD200TDriver:
    """Low-level MD200T RS485 serial driver."""

    def __init__(
        self,
        port='/dev/ttyUSB0',
        baudrate=57600,
        robot_id=1,
        max_rpm=4000,
    ):
        self.port = port
        self.baudrate = baudrate
        self.robot_id = robot_id
        self.max_rpm = max_rpm
        self.serial_port = None
        self.lock = threading.Lock()

        self.RMID = 183
        self.TMID = 184
        self.PID_PNT_VEL_CMD = 207
        self.PID_TQ_OFF = 5
        self.PID_MAIN_BC = 124

    def connect(self):
        if serial is None:
            print('[ERROR] pyserial is not installed. Install python3-serial.')
            return False

        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
            )
            return self.serial_port.is_open
        except Exception as exc:
            print(f'[ERROR] MD200T connection failed: {exc}')
            return False

    def _calculate_checksum(self, packet_bytes):
        total_sum = sum(packet_bytes) & 0xFF
        return (~total_sum + 1) & 0xFF

    def send_param(self, pid, data, byte_size=1):
        if not self.serial_port or not self.serial_port.is_open:
            return False

        if byte_size == 1:
            data_payload = bytes([data])
        elif byte_size == 2:
            data_payload = struct.pack('<h', data)
        else:
            raise ValueError('byte_size must be 1 or 2')

        packet_no_chk = (
            bytes([self.RMID, self.TMID, self.robot_id, pid, byte_size])
            + data_payload
        )
        checksum = self._calculate_checksum(packet_no_chk)

        with self.lock:
            try:
                self.serial_port.write(packet_no_chk + bytes([checksum]))
                time.sleep(0.05)
                return True
            except Exception as exc:
                print(f'[ERROR] parameter send failed pid={pid}: {exc}')
                return False

    def initialize_motor(self):
        if not self.serial_port or not self.serial_port.is_open:
            return False

        self.send_param(81, 0, 1)
        self.send_param(92, 0, 1)
        self.send_param(self.PID_MAIN_BC, 1, 1)
        return True

    def send_rpm_command(self, left_rpm, right_rpm):
        if not self.serial_port or not self.serial_port.is_open:
            return False

        left_rpm_clamped = max(
            min(int(left_rpm), self.max_rpm),
            -self.max_rpm,
        )
        right_rpm_clamped = max(
            min(int(right_rpm), self.max_rpm),
            -self.max_rpm,
        )

        motor1_bytes = struct.pack('<h', left_rpm_clamped)
        motor2_bytes = struct.pack('<h', right_rpm_clamped)

        data_payload = (
            bytes([1]) + motor1_bytes + bytes([1]) + motor2_bytes + bytes([0])
        )
        packet_no_chk = (
            bytes([
                self.RMID,
                self.TMID,
                self.robot_id,
                self.PID_PNT_VEL_CMD,
                7,
            ])
            + data_payload
        )
        checksum = self._calculate_checksum(packet_no_chk)

        with self.lock:
            try:
                self.serial_port.write(packet_no_chk + bytes([checksum]))
                self.serial_port.read(self.serial_port.in_waiting)
                return True
            except Exception as exc:
                print(f'[ERROR] RPM command send failed: {exc}')
                return False

    def stop_motor(self):
        if not self.serial_port or not self.serial_port.is_open:
            return

        self.send_rpm_command(0, 0)
        time.sleep(0.1)
        self.send_param(self.PID_MAIN_BC, 0, 1)

    def disconnect(self):
        if self.serial_port and self.serial_port.is_open:
            self.stop_motor()
            time.sleep(0.1)
            self.serial_port.close()
