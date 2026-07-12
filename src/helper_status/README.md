# helper_status

`/control/odom`을 받아 Helper Robotics의 `RobotStatus` 메시지로 변환해
`/control/status`에 발행합니다. 현재 배터리와 일부 상태 필드는 고정된 데모값입니다.

## 빌드 및 실행

```bash
cd ~/workspace/helper_robotics
colcon build --symlink-install --packages-select helper_msgs helper_status
source install/setup.bash
ros2 run helper_status status_node
```

```text
/control/odom (nav_msgs/Odometry)
  -> status_node
  -> /control/status (helper_msgs/RobotStatus, 10 Hz)
```

확인:

```bash
ros2 topic echo /control/status
```
