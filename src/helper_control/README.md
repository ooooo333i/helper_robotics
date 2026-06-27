# helper_control

`helper_control`은 Nav2 또는 테스트 노드가 만든 속도 명령을 안전하게 제한하고,
차동 구동 RPM으로 변환해 MD200T 모터 드라이버에 전달하는 ROS 2 패키지입니다.
현재 odometry는 엔코더 피드백이 아니라 명령 RPM을 적분한 open-loop 값입니다.

## 빌드와 실행

```bash
cd ~/workspace/helper_robotics
colcon build --symlink-install --packages-select helper_msgs helper_control
source install/setup.bash
```

실제 모터와 safety gate를 함께 실행합니다.

```bash
ros2 launch helper_control motor_driver.launch.py \
  serial_port:=/dev/ttyUSB1
```

환경 변수로 포트를 지정할 수도 있습니다. `AMR_MOTOR_DRIVER_PORT`가
`MOTOR_DRIVER_PORT`보다 우선합니다.

```bash
export AMR_MOTOR_DRIVER_PORT=/dev/serial/by-id/<motor-device>
ros2 launch helper_control motor_driver.launch.py
```

모터 출력 없이 장애물 정지 로직을 시험하려면 다음 launch를 사용합니다.

```bash
ros2 launch helper_control motor_obstacle_dry_run_test.launch.py
```

LiDAR, depth 인식, fusion과 모터를 한 번에 실행하는 별도 bringup도 있습니다.

```bash
ros2 launch helper_control obstacle_safety_bringup.launch.py \
  cmd_vel_topic:=/control/cmd_vel \
  stop_on_unknown:=false
```

## 노드와 입출력

### `cmd_vel_safety_gate`

| 구분 | 이름 | 타입 | 내용 |
|---|---|---|---|
| 입력 | `/control/cmd_vel_smoothed` | `geometry_msgs/msg/Twist` | Nav2 velocity smoother가 만든 속도 |
| 입력 | `/planning/behavior_state` | `std_msgs/msg/String` | `run`, `stop`, `overcome`, `avoid` |
| 출력 | `/control/cmd_vel_safe` | `geometry_msgs/msg/Twist` | 모터로 전달할 최종 속도 |

`stop`이면 0인 `Twist`를 계속 발행합니다. `run`, `overcome`, `avoid`가 오면
차단을 해제하고, 속도 입력이 기본 0.5초 동안 끊겨도 0을 발행합니다.

### `motor_driver`

| 구분 | 이름 | 타입 | 내용 |
|---|---|---|---|
| 입력 | `/control/cmd_vel_safe` | `geometry_msgs/msg/Twist` | `linear.x` [m/s], `angular.z` [rad/s] 사용 |
| 입력 | `/perception/obstacle/fused` 등 | `helper_msgs/msg/ObstacleDecision` | 선택적인 모터단 장애물 안전 정지 |
| 출력 | `/control/odom` | `nav_msgs/msg/Odometry` | 명령 RPM 적분 odometry |
| 출력 | `/tf` | `geometry_msgs/msg/TransformStamped` | `odom -> base_link` 변환 |
| 장치 출력 | MD200T serial | binary packet | 좌·우 목표 RPM |

기본 장애물 입력 목록은 `/perception/obstacle/fused`,
`/perception/obstacle/range`, `/perception/obstacle/depth`입니다.
`motor_driver.launch.py`는 `safety_stop_enabled=false`로 실행하므로 일반
Nav2 주행에서는 behavior safety gate가 최종 정지를 담당합니다.

주요 파라미터는 `config/motor_driver.yaml`에 있습니다.

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `serial_port` | `/dev/ttyUSB0` | MD200T serial 장치 |
| `baud_rate` | `57600` | serial baud rate |
| `wheel_radius` | `0.065` | 바퀴 반지름 [m] |
| `track_width` | `0.190` | 좌우 바퀴 간격 [m] |
| `gear_ratio` | `49.0` | 감속비 |
| `max_linear_vel` | `1.0` | 선속도 제한 [m/s] |
| `max_angular_vel` | `1.1` | 각속도 제한 [rad/s] |
| `cmd_timeout` | `0.5` | 명령 watchdog [s] |
| `dry_run` | `false` | serial 출력 없이 계산/로그만 수행 |
| `safety_stop_enabled` | `false` | 모터단 장애물 정지 활성화 |
| `stop_on_unknown` | `false` | stale/unknown 장애물 입력 시 정지 |

### 데모 입력 노드

- `fake_scan`: 장애물 없는 `sensor_msgs/msg/LaserScan`을
  `/perception/scan/filtered`에 10 Hz로 발행합니다.
- `fake_odom`: 고정된 `nav_msgs/msg/Odometry`와 `odom -> base_link` TF를
  발행합니다.

```bash
ros2 run helper_control fake_scan
ros2 run helper_control fake_odom
```

## 구현 로직

```text
/control/cmd_vel_smoothed
  -> behavior/timeout safety gate
  -> /control/cmd_vel_safe
  -> 선·각속도 제한
  -> 차동 구동 inverse kinematics
  -> RPM 가감속 ramp와 방향/scale 보정
  -> MD200T serial packet
```

`motor_driver`는 속도 명령이 끊기면 watchdog으로 목표 RPM을 0으로 만들고
정지/브레이크 명령을 보냅니다. 장애물 안전 정지가 활성화된 경우 fresh한 입력
중 하나라도 `decision="obstacle"`이면 정지하며, `stop_on_unknown=true`이면
입력이 없거나 `unknown`이어도 정지합니다.

Odometry는 현재 목표 RPM을 forward kinematics로 다시 선·각속도로 바꾸어
적분합니다. 바퀴 미끄러짐이나 실제 회전량을 측정하지 않으므로 정밀 위치 추정에는
엔코더 기반 odometry가 필요합니다.

## 확인 명령

```bash
ros2 topic echo /control/cmd_vel_safe
ros2 topic echo /control/odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 param get /motor_driver_node serial_port
```
