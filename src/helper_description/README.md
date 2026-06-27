# helper_description

`helper_description`은 helper robot의 차체, 바퀴, 전·후방 LiDAR, depth camera
좌표계를 정의하는 URDF/Xacro 패키지입니다.

## 빌드와 실행

```bash
cd ~/workspace/helper_robotics
colcon build --symlink-install --packages-select helper_description
source install/setup.bash
ros2 launch helper_description display.launch.py
```

## 입력과 출력

이 패키지에는 ROS topic 입력이 없습니다. `display.launch.py`가
`urdf/helper_robot.urdf.xacro`를 `xacro`로 변환해
`robot_state_publisher`의 `robot_description` 파라미터로 전달합니다.

출력은 `/tf_static`의 고정 TF입니다.

```text
base_link
├── left_wheel
├── right_wheel
├── laser_front
├── laser_rear
└── depth_camera_link
```

주요 장착 위치는 다음과 같습니다.

| child frame | parent | 위치/회전 |
|---|---|---|
| `laser_front` | `base_link` | xyz `(0.149, 0, 0.2772)`, yaw `π` |
| `laser_rear` | `base_link` | xyz `(-0.149, 0, 0.2772)`, yaw `π` |
| `depth_camera_link` | `base_link` | xyz `(0.1847, 0, 0.1889)`, pitch `0.95993` |

`map -> odom`과 `odom -> base_link`는 이 패키지가 만들지 않습니다.
각각 SLAM/AMCL과 motor odometry가 담당합니다.

## 구현 로직

Xacro 안의 macro로 box/cylinder inertia와 좌우 구동 바퀴를 정의하고,
센서는 `base_link`에 fixed joint로 연결합니다. launch는 생성된
`robot_description`만 `robot_state_publisher`에 넘기는 최소 구성입니다.

## 확인 명령

```bash
ros2 param get /robot_state_publisher robot_description
ros2 run tf2_ros tf2_echo base_link laser_front
ros2 run tf2_ros tf2_echo base_link depth_camera_link
```
