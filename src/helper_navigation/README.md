# helper_navigation

SLAM, 저장 지도 localization(AMCL), Nav2 자율주행, behavior 기반 정지·재개를
연결하는 패키지입니다.

## 빌드

```bash
cd ~/workspace/helper_robotics
colcon build --symlink-install --packages-up-to helper_navigation
source install/setup.bash
```

## 실행

장치 없는 데모:

```bash
ros2 launch helper_navigation behavior_nav2_demo.launch.py rviz:=true
```

실제 로봇 SLAM:

```bash
source ~/workspace/helper_robotics/config/usb_ports.env
ros2 launch helper_navigation slam_bringup.launch.py motor:=true rviz:=true
```

모터 없이 센서·SLAM만 먼저 확인하려면 `motor:=false`로 실행합니다.

지도 저장:

```bash
ros2 run nav2_map_server map_saver_cli -f \
  ~/workspace/helper_robotics/src/helper_navigation/maps/helper_map
```

저장 지도 자율주행:

```bash
source ~/workspace/helper_robotics/config/usb_ports.env
ros2 launch helper_navigation map_navigation.launch.py \
  map:=$HOME/workspace/helper_robotics/src/helper_navigation/maps/helper_maps.yaml \
  motor:=true rviz:=true
```

## 흐름

```text
/planning/goal_pose -> behavior_manager -> Nav2
  -> /control/cmd_vel -> velocity_smoother
  -> /control/cmd_vel_smoothed -> safety gate -> motor

/planning/behavior_cmd -> behavior_manager
  -> /planning/behavior_state (run/stop/overcome/avoid)
```

RViz에서는 `2D Pose Estimate`로 초기 위치를 설정하고 `2D Goal Pose`로 목표를
보냅니다. 수동 behavior 테스트:

```bash
ros2 topic pub --times 3 /planning/behavior_cmd std_msgs/msg/String "{data: stop}"
ros2 topic pub --times 3 /planning/behavior_cmd std_msgs/msg/String "{data: run}"
```

상태 확인:

```bash
ros2 topic echo /planning/behavior_state
ros2 topic echo /control/odom --once
ros2 run tf2_ros tf2_echo map base_link
ros2 action list
```

주요 설정 파일:

- `config/helper_slam_params.yaml`: SLAM Toolbox 설정
- `config/helper_nav2_params.yaml`: AMCL, planner, controller, costmap 설정
- `config/helper_nav2_fake_params.yaml`: 장치 없는 데모용 Nav2 설정
