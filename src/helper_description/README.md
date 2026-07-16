# helper_description

로봇 차체, 바퀴, 전·후방 LiDAR, depth camera의 URDF/Xacro와 고정 TF를
제공합니다. `map→odom`과 `odom→base_link`는 이 패키지의 범위가 아닙니다.

## 빌드 및 실행

```bash
cd ~/helper_robotics
colcon build --symlink-install --packages-select helper_description
source install/setup.bash
ros2 launch helper_description display.launch.py
```

주요 TF 구조:

```text
base_link
├── left_wheel / right_wheel
├── laser_front / laser_rear
└── depth_camera_link -> camera_link
```

확인:

```bash
ros2 param get /robot_state_publisher robot_description
ros2 run tf2_ros tf2_echo base_link laser_front
ros2 run tf2_ros tf2_echo base_link depth_camera_link
```
