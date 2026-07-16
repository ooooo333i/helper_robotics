# helper_control

Nav2 속도 명령에 behavior/timeout 안전 정지를 적용하고, 차동 구동 RPM으로
변환해 MD200T 모터 드라이버로 전달합니다. MD200T의 실제 좌·우 RPM feedback을
우선 사용하여 wheel odometry를 생성하고, feedback timeout 시 명령 RPM을
이용해 odometry를 계속 발행합니다.

## 빌드

```bash
cd ~/helper_robotics
colcon build --symlink-install --packages-select helper_msgs helper_control
source install/setup.bash
```

## 실행

모터 드라이버와 safety gate:

```bash
ros2 launch helper_control motor_driver.launch.py serial_port:=/dev/ttyUSB0
```

포트 환경 변수를 사용할 수도 있습니다.

```bash
export AMR_MOTOR_DRIVER_PORT=/dev/serial/by-id/<motor-device>
ros2 launch helper_control motor_driver.launch.py
```

모터 출력 없는 장애물 정지 테스트:

```bash
ros2 launch helper_control motor_obstacle_dry_run_test.launch.py
```

LiDAR·depth·fusion·모터 통합 실행:

```bash
ros2 launch helper_control obstacle_safety_bringup.launch.py
```

키보드 수동 주행:

```bash
ros2 run helper_control keyboard_teleop
```

## 주요 입출력

```text
/control/cmd_vel_smoothed + /planning/behavior_state
  -> cmd_vel_safety_gate
  -> /control/cmd_vel_safe
  -> motor_driver
  -> MD200T + /control/odom + odom→base_link TF
```

설정은 `config/motor_driver.yaml`에서 변경합니다. 주요 설정 항목은 serial port,
wheel radius, track width, gear ratio, 속도·RPM 제한, feedback timeout 및 좌·우
모터 방향/scale입니다.

```bash
ros2 topic echo /control/cmd_vel_safe
ros2 topic echo /control/odom
ros2 run tf2_ros tf2_echo odom base_link
```
