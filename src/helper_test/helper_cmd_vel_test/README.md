# helper_cmd_vel_test

`helper_cmd_vel_test`는 모터 bringup과 장애물 safety gate를 검증하기 위한
테스트 publisher 모음입니다. 기본 출력 `/control/cmd_vel_safe`는 실제 모터가
구독하므로 바퀴를 띄우거나 비상 정지 수단을 확보한 뒤 사용하십시오.

## 빌드

```bash
cd ~/workspace/helper_robotics
colcon build --symlink-install --packages-select helper_msgs helper_cmd_vel_test
source install/setup.bash
```

## 입출력 형식

모든 노드는 입력 topic이 없고 다음 중 하나를 발행합니다.

| 노드 | 출력 | 타입 |
|---|---|---|
| `cmd_vel_test` | `/control/cmd_vel_safe` | `geometry_msgs/msg/Twist` |
| `cmd_vel_distance_test` | `/control/cmd_vel_safe` | `geometry_msgs/msg/Twist` |
| `cmd_vel_turn_test` | `/control/cmd_vel_safe` | `geometry_msgs/msg/Twist` |
| `cmd_vel_stop` | `/control/cmd_vel_safe` | `geometry_msgs/msg/Twist` |
| `obstacle_decision_test` | `/perception/obstacle/fused` | `helper_msgs/msg/ObstacleDecision` |

속도 명령은 `Twist.linear.x` [m/s]와 `Twist.angular.z` [rad/s]만 사용합니다.

## 실행 명령

일정 속도:

```bash
ros2 run helper_cmd_vel_test cmd_vel_test --ros-args \
  -p linear_x:=0.1 -p angular_z:=0.0
```

전진/정지/좌회전/정지/우회전/정지 반복:

```bash
ros2 run helper_cmd_vel_test cmd_vel_test --ros-args \
  -p mode:=sequence -p linear_x:=0.1 -p angular_z:=0.3 \
  -p step_duration:=2.0
```

open-loop 0.5 m 전진:

```bash
ros2 run helper_cmd_vel_test cmd_vel_distance_test
```

open-loop 90° 제자리 회전:

```bash
ros2 run helper_cmd_vel_test cmd_vel_turn_test --ros-args \
  -p mode:=spin -p angular_z:=0.3 -p target_yaw_deg:=90.0
```

주행하며 우측으로 90° arc:

```bash
ros2 run helper_cmd_vel_test cmd_vel_turn_test --ros-args \
  -p mode:=arc -p linear_x:=0.1 -p angular_z:=-0.3 \
  -p target_yaw_deg:=90.0
```

즉시 정지 메시지 반복 발행:

```bash
ros2 run helper_cmd_vel_test cmd_vel_stop
```

장애물 상태를 `clear 4초 -> obstacle 3초`로 반복:

```bash
ros2 run helper_cmd_vel_test obstacle_decision_test
```

launch 실행:

```bash
ros2 launch helper_cmd_vel_test cmd_vel_constant.launch.py
ros2 launch helper_cmd_vel_test cmd_vel_sequence.launch.py
ros2 launch helper_cmd_vel_test obstacle_decision_sequence.launch.py
```

`cmd_vel_forward_test.launch.py`는 기본 출력이 다른 launch와 달리
`/control/cmd_vel_test`입니다.

```bash
ros2 launch helper_cmd_vel_test cmd_vel_forward_test.launch.py \
  topic:=/control/cmd_vel_test linear_x:=0.03
```

## 현재 로직

- `cmd_vel_test`: constant 또는 6단계 sequence를 timer로 무한 반복합니다.
- `cmd_vel_distance_test`: `distance / linear_x`만큼 명령한 뒤 0을 반복 발행합니다.
- `cmd_vel_turn_test`: `radians(target_yaw_deg) / abs(angular_z)`만큼 명령한 뒤
  정지합니다.
- `cmd_vel_stop`: 0인 `Twist`를 지정 횟수만 발행하고 종료합니다.
- `obstacle_decision_test`: `clear`, `obstacle`, `unknown`, `sequence` mode로
  가짜 `ObstacleDecision`을 만듭니다.

거리/각도 테스트는 odometry나 encoder feedback을 사용하지 않는 시간 기반
open-loop 테스트이므로 실제 이동 오차가 발생합니다.

## 주요 파라미터

| 파라미터 | 기본값 | 적용 |
|---|---:|---|
| `topic` | `/control/cmd_vel_safe` | 전체 |
| `publish_rate` | 노드별 10 또는 20 Hz | 전체 |
| `linear_x` | 노드별 상이 | 속도 노드 |
| `angular_z` | 노드별 상이 | 속도 노드 |
| `distance` | `0.5` m | distance |
| `target_yaw_deg` | `90.0`° | turn |
| `mode` | 노드별 상이 | constant/sequence 또는 spin/arc |
| `stop_publish_count` | 노드별 10 또는 20 | distance/turn/stop |

확인:

```bash
ros2 topic echo /control/cmd_vel_safe
ros2 topic echo /perception/obstacle/fused
```
