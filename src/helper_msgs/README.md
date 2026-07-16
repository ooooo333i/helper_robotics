# helper_msgs

Helper Robotics 패키지 사이에서 공유하는 ROS 2 custom message를 정의합니다.
실행 노드는 없으며 빌드 시 message type support가 생성됩니다.

## 빌드 및 확인

```bash
cd ~/helper_robotics
colcon build --symlink-install --packages-select helper_msgs
source install/setup.bash

ros2 interface show helper_msgs/msg/ObstacleDecision
ros2 interface show helper_msgs/msg/RobotStatus
ros2 interface show helper_msgs/msg/LiftState
```

| 메시지 | 용도 |
|---|---|
| `ObstacleDecision` | LiDAR/depth/fusion 장애물 판단 |
| `RobotStatus` | 위치, 속도, 배터리, 동작 상태 |
| `LiftState` | 리프트 높이와 동작 상태 |

장애물 메시지 발행 예시:

```bash
ros2 topic pub --once /perception/obstacle/fused \
  helper_msgs/msg/ObstacleDecision \
  "{obstacle_type: test, decision: obstacle, distance: 0.3, height: 0.1, is_dynamic: false}"
```
