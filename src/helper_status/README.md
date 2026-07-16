# helper_status

`/control/odom`을 받아 Helper Robotics의 `RobotStatus` 메시지로 변환해
`/control/status`에 10 Hz로 발행합니다. odometry의 x, y 위치를 상태 메시지에
반영하고 로봇 ID, 배터리, 현재 action 및 상태 정보를 함께 제공합니다.

## 빌드 및 실행

```bash
cd ~/helper_robotics
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
