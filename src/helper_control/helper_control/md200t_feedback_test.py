import argparse
import time

from helper_control.md200t_driver import MD200TDriver


def build_parser():
    parser = argparse.ArgumentParser(
        description='Read MD200T PID 193/200 motor feedback for testing.'
    )
    parser.add_argument(
        '--port',
        default=(
            '/dev/serial/by-id/'
            'usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0'
        ),
        help='Serial port for the MD200T driver.',
    )
    parser.add_argument('--baud-rate', type=int, default=57600)
    parser.add_argument('--driver-id', type=int, default=1)
    parser.add_argument('--max-rpm', type=int, default=4000)
    parser.add_argument('--read-timeout', type=float, default=0.2)
    parser.add_argument('--duration', type=float, default=5.0)
    parser.add_argument('--rate', type=float, default=5.0)
    parser.add_argument(
        '--pid',
        type=int,
        choices=[193, 200],
        default=None,
        help='Read only one MAIN_DATA PID instead of both motors.',
    )
    parser.add_argument(
        '--return-type',
        type=int,
        choices=[0, 1, 2],
        default=None,
        help='Send RPM command with return type and print raw response.',
    )
    parser.add_argument(
        '--pnt-main-data',
        action='store_true',
        help='Request return type 2 and parse PID 210 PNT main data.',
    )
    parser.add_argument(
        '--motor1-rpm',
        type=int,
        default=0,
        help='Raw motor1 RPM command. Keep 0 for read-only testing.',
    )
    parser.add_argument(
        '--motor2-rpm',
        type=int,
        default=0,
        help='Raw motor2 RPM command. Keep 0 for read-only testing.',
    )
    parser.add_argument(
        '--initialize',
        action='store_true',
        help='Run normal motor initialization before testing.',
    )
    parser.add_argument(
        '--no-stop-on-exit',
        action='store_true',
        help='Do not send a stop command when the test exits.',
    )
    return parser


def print_feedback(feedback):
    motor1 = feedback['motor1']
    motor2 = feedback['motor2']
    print(
        'motor1 '
        f'rpm={motor1["rpm"]:6d} '
        f'ref={motor1["ref_rpm"]:6d} '
        f'cur={motor1["current_raw"]:5d} '
        f'out={motor1["control_output"]:6d} '
        f'type={motor1["control_type"]} '
        f'st={motor1["controller_status"]:3d} '
        '| motor2 '
        f'rpm={motor2["rpm"]:6d} '
        f'ref={motor2["ref_rpm"]:6d} '
        f'cur={motor2["current_raw"]:5d} '
        f'out={motor2["control_output"]:6d} '
        f'type={motor2["control_type"]} '
        f'st={motor2["controller_status"]:3d}',
        flush=True,
    )


def print_main_data(pid, data):
    print(
        f'pid={pid} '
        f'rpm={data["rpm"]:6d} '
        f'ref={data["ref_rpm"]:6d} '
        f'cur={data["current_raw"]:5d} '
        f'out={data["control_output"]:6d} '
        f'type={data["control_type"]} '
        f'st={data["controller_status"]:3d}',
        flush=True,
    )


def print_pnt_main_data(data):
    print(
        'pid=210 '
        f'motor1_rpm={data["motor1_rpm"]:6d} '
        f'motor1_cur={data["motor1_current_raw"]:5d} '
        f'motor1_st={data["motor1_status"]:3d} '
        f'motor1_pos={data["motor1_position"]:10d} '
        '| '
        f'motor2_rpm={data["motor2_rpm"]:6d} '
        f'motor2_cur={data["motor2_current_raw"]:5d} '
        f'motor2_st={data["motor2_status"]:3d} '
        f'motor2_pos={data["motor2_position"]:10d}',
        flush=True,
    )


def format_hex(data):
    if not data:
        return '<no bytes>'
    return ' '.join(f'{value:02X}' for value in data)


def main():
    args = build_parser().parse_args()
    driver = MD200TDriver(
        port=args.port,
        baudrate=args.baud_rate,
        robot_id=args.driver_id,
        max_rpm=args.max_rpm,
    )

    if not driver.connect():
        print(f'connect failed: {driver.last_error or "unknown"}')
        return 1

    command_requested = args.motor1_rpm != 0 or args.motor2_rpm != 0
    if args.pnt_main_data:
        args.return_type = 2

    try:
        if args.initialize or command_requested:
            print('initializing motor driver...')
            if not driver.initialize_motor():
                print('initialize failed')
                return 1

        if command_requested or args.return_type is not None:
            print(
                'sending raw RPM command: '
                f'motor1={args.motor1_rpm}, motor2={args.motor2_rpm}, '
                f'return_type={args.return_type or 0}'
            )
            if not driver.send_rpm_command(
                args.motor1_rpm,
                args.motor2_rpm,
                return_type=args.return_type or 0,
                clear_response=args.return_type is None,
            ):
                print('RPM command failed')
                return 1
        else:
            print('read-only mode: no RPM command will be sent')

        period = 1.0 / max(args.rate, 0.1)
        end_time = time.monotonic() + max(args.duration, 0.0)
        while time.monotonic() < end_time:
            if args.pnt_main_data:
                driver.send_rpm_command(
                    args.motor1_rpm,
                    args.motor2_rpm,
                    return_type=2,
                    clear_response=False,
                )
                data = driver.read_pnt_main_data_response(
                    timeout=args.read_timeout
                )
                if data is None:
                    print('pid 210 parse failed', flush=True)
                else:
                    print_pnt_main_data(data)
            elif args.return_type is not None:
                driver.send_rpm_command(
                    args.motor1_rpm,
                    args.motor2_rpm,
                    return_type=args.return_type,
                    clear_response=False,
                )
                raw = driver.read_raw_available(timeout=args.read_timeout)
                print(f'raw: {format_hex(raw)}', flush=True)
            elif args.pid is not None:
                data = driver.read_main_data(args.pid, timeout=args.read_timeout)
                if data is None:
                    print(f'pid {args.pid} read failed', flush=True)
                else:
                    print_main_data(args.pid, data)
            else:
                feedback = driver.read_motor_feedback(timeout=args.read_timeout)
                if feedback is None:
                    print('feedback read failed', flush=True)
                else:
                    print_feedback(feedback)
            time.sleep(period)

        return 0
    finally:
        if not args.no_stop_on_exit:
            driver.send_rpm_command(0, 0)
            time.sleep(0.05)
        driver.disconnect()


if __name__ == '__main__':
    raise SystemExit(main())
