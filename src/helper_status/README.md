# helper_status

`helper_status`는 odometry를 간단한 `RobotStatus`로 변환해 발행합니다.

## 빌드와 실행

```bash
cd ~/workspace/helper_robotics
colcon build --symlink-install --packages-select helper_msgs helper_status
source install/setup.bash
ros2 run helper_status status_node
```

## 입력과 출력

| 구분 | topic | 타입 | 사용 필드 |
|---|---|---|---|
| 입력 | `/control/odom` | `nav_msgs/msg/Odometry` | `pose.pose.position.x`, `y` |
| 출력 | `/control/status` | `helper_msgs/msg/RobotStatus` | 전체 status 메시지 |

출력은 10 Hz이며 현재 값은 다음과 같이 채워집니다.

- `robot_id="robot_01"`
- `x`, `y`: 마지막 odometry 위치
- `theta=0.0`: quaternion 변환은 아직 구현되지 않음
- `linear_velocity`, `angular_velocity`: 메시지 기본값 `0.0`
- `battery_percent=80.0`, `current_action="idle"`, `status="normal"`: 고정값

## 구현 로직

odometry callback은 최신 x/y만 저장하고, 별도 timer가 저장값과 고정 상태값으로
`RobotStatus`를 구성해 발행합니다. 따라서 현재 노드는 실제 배터리나 동작 상태를
수집하는 완성형 상태 집계기가 아니라 인터페이스 연결용 초기 구현입니다.

```bash
ros2 topic echo /control/status
```
