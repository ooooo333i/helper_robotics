# helper_msgs

`helper_msgs`는 helper robot 패키지 사이에서 사용하는 custom ROS 2 message를
정의합니다. 실행 노드는 없으며 빌드 시 message type support를 생성합니다.

## 빌드와 확인

```bash
cd ~/workspace/helper_robotics
colcon build --symlink-install --packages-select helper_msgs
source install/setup.bash

ros2 interface show helper_msgs/msg/ObstacleDecision
ros2 interface show helper_msgs/msg/RobotStatus
ros2 interface show helper_msgs/msg/LiftState
```

## 메시지 형식

### `ObstacleDecision`

```text
string obstacle_type
string decision
float64 distance
float64 height
bool is_dynamic
```

- `obstacle_type`: `range`, `depth`, `fused`, `test` 등 판단 출처
- `decision`: 현재 코드에서 `obstacle`, `clear`, `unknown` 사용
- `distance`: 장애물 거리 [m], 유효값이 없으면 주로 `inf`
- `height`: 추정 장애물 높이 [m]
- `is_dynamic`: 동적 장애물 여부

예시:

```bash
ros2 topic pub --once /perception/obstacle/fused \
  helper_msgs/msg/ObstacleDecision \
  "{obstacle_type: test, decision: obstacle, distance: 0.3, height: 0.1, is_dynamic: false}"
```

### `RobotStatus`

```text
string robot_id
float64 x
float64 y
float64 theta
float64 linear_velocity
float64 angular_velocity
float32 battery_percent
string current_action
string status
```

위치 단위는 m/rad, 속도 단위는 m/s와 rad/s, 배터리는 percent입니다.
현재 `helper_status` 구현은 일부 필드를 고정값 또는 기본값으로 채웁니다.

### `LiftState`

```text
string lift_state
float64 height
bool is_moving
```

리프트 상태, 높이, 동작 여부를 표현합니다. 현재 저장소에는 이 메시지를 발행하거나
구독하는 노드가 없습니다.
