# helper_navigation

`helper_navigation`은 helper robot의 SLAM, 저장 지도 localization, Nav2 주행,
behavior 기반 정지/재개를 연결합니다.

## 빌드

```bash
cd ~/workspace/helper_robotics
colcon build --symlink-install --packages-select \
  helper_msgs helper_description helper_control helper_perception helper_navigation
source install/setup.bash
```

## 실행 시나리오

### 실제 장치 없는 Nav2 데모

```bash
ros2 launch helper_navigation behavior_nav2_demo.launch.py rviz:=true
```

fake scan과 `/control/cmd_vel_safe` 적분 odometry를 사용합니다. RViz의
`2D Goal Pose`가 `/planning/goal_pose`로 설정되어 있으며 실제 모터는 움직이지
않습니다.

### 실제 로봇 SLAM과 지도 저장

USB 포트 환경을 먼저 불러온 뒤 실행합니다.

```bash
cd ~/workspace/helper_robotics
source /opt/ros/humble/setup.bash
source install/setup.bash
source config/usb_ports.env

ros2 launch helper_navigation slam_bringup.launch.py \
  motor:=true \
  rviz:=true
```

`slam_bringup`은 전방 LiDAR, robot description, motor/safety gate,
SLAM Toolbox, Nav2, behavior manager를 실행합니다. 현재
`mapping_navigation.launch.py` 자체에는 LiDAR driver가 없으므로 실기 mapping
진입점으로는 `slam_bringup.launch.py`를 사용합니다.

지도를 저장합니다.

```bash
ros2 run nav2_map_server map_saver_cli -f \
  ~/workspace/helper_robotics/src/helper_navigation/maps/helper_map
```

### 저장된 지도에서 실제 주행

```bash
ros2 launch helper_navigation map_navigation.launch.py \
  map:=$HOME/workspace/helper_robotics/src/helper_navigation/maps/helper_map.yaml \
  front_lidar_port:=${AMR_FRONT_LIDAR_PORT} \
  motor_port:=${AMR_MOTOR_DRIVER_PORT} \
  motor:=true \
  rviz:=true
```

이 launch는 전방 LiDAR, robot description, motor/safety gate, AMCL/map server,
Nav2, behavior manager를 포함합니다. depth 인식과 자동 behavior 판단이 필요하면
depth camera driver를 먼저 실행한 뒤 별도 터미널에서 다음을 추가합니다.

```bash
ros2 launch helper_perception depth_obstacle.launch.py
ros2 launch helper_perception perception_behavior_gate.launch.py
```

`perception_behavior_gate`의 기본 path 입력은 설정 파일 기준 `/local_plan`입니다.
실행 중 실제 Nav2가 해당 topic을 발행하는지 반드시 확인하십시오.

`depth_obstacle.launch.py`는 두 `sensor_msgs/msg/PointCloud2`를 발행합니다.
`/perception/depth/obstacle_points`는 local costmap의 `depth_mark`,
`/perception/depth/clearing_points`는 `depth_clear` source로 연결됩니다.
Depth launch를 실행하지 않으면 두 source에는 데이터가 없고 LiDAR scan만
사용됩니다.

## 전체 데이터 흐름

```text
LiDAR/depth + Nav2 local path + odom
  -> helper_perception
  -> /planning/behavior_cmd
  -> behavior_manager
       ├── NavigateToPose action 제어
       ├── /planning/behavior_state
       └── /speed_limit

/planning/goal_pose
  -> behavior_manager
  -> Nav2 planner/controller
  -> /control/cmd_vel
  -> velocity_smoother
  -> /control/cmd_vel_smoothed
  -> cmd_vel_safety_gate
  -> /control/cmd_vel_safe
  -> motor_driver
```

TF 책임은 다음과 같습니다.

```text
map -> odom                  SLAM Toolbox 또는 AMCL
odom -> base_link           motor_driver의 open-loop odometry
base_link -> sensor frames  robot_state_publisher + URDF
```

## 노드별 입출력

### `behavior_manager`

| 구분 | 이름 | 타입 | 내용 |
|---|---|---|---|
| 입력 | `/planning/goal_pose` | `geometry_msgs/msg/PoseStamped` | map/odom frame 목표 pose |
| 입력 | `/planning/behavior_cmd` | `std_msgs/msg/String` | `run`, `stop`, `overcome`, `avoid` |
| 출력 | `/planning/behavior_state` | `std_msgs/msg/String` | 현재 behavior, 기본 2 Hz |
| 출력 | `/speed_limit` | `nav2_msgs/msg/SpeedLimit` | `overcome`의 percentage 제한 |
| action client | `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 목표 전송/취소 |
| service client | local/global clear service | `nav2_msgs/srv/ClearEntireCostmap` | costmap clear용 |

현재 behavior 로직:

- `run`: speed limit을 해제하고 behavior 때문에 멈춘 기존 goal을 재전송합니다.
- `stop`: 현재 Nav2 goal을 cancel합니다. 동시에 safety gate가 최종 속도를
  0으로 차단합니다.
- `overcome`: 기본 80% speed limit을 발행하고 goal을 재개합니다.
- `avoid`: speed limit을 해제하고 goal을 유지/재개합니다.

코드에는 costmap clear와 goal restart 함수가 있지만 현재 `handle_avoid()`에서는
호출하지 않습니다. 즉, 현재 `avoid`가 즉시 costmap clear/replan을 수행한다고
가정하면 안 됩니다. 대신 LiDAR/Depth 장애물이 costmap에 표시된 상태에서 DWB
local planner가 회피 속도를 계산하고, 기본 Nav2 BT
`navigate_to_pose_w_replanning_and_recovery.xml`이 global path를 주기적으로
재계산합니다.

### `demo_cmd_vel_odom`

| 입력 | 출력 |
|---|---|
| `/control/cmd_vel_safe` (`geometry_msgs/msg/Twist`) | `/control/odom` (`nav_msgs/msg/Odometry`), `odom -> base_link` TF |

속도를 최대 선속도 0.35 m/s, 각속도 1.2 rad/s로 제한하고 시간 적분합니다.
0.5초간 새 명령이 없으면 정지합니다. 데모 전용이며 실제 센서 odometry가 아닙니다.

## 주요 topic

| topic | 타입 | 역할 |
|---|---|---|
| `/planning/goal_pose` | `geometry_msgs/msg/PoseStamped` | behavior manager로 보낼 목표 |
| `/planning/behavior_cmd` | `std_msgs/msg/String` | perception/VDA5050의 behavior 요청 |
| `/planning/behavior_state` | `std_msgs/msg/String` | 확정된 현재 behavior |
| `/control/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 controller 출력 |
| `/control/cmd_vel_smoothed` | `geometry_msgs/msg/Twist` | velocity smoother 출력 |
| `/control/cmd_vel_safe` | `geometry_msgs/msg/Twist` | safety gate 이후 최종 속도 |
| `/control/odom` | `nav_msgs/msg/Odometry` | 로봇 odometry |
| `/perception/scan/filtered` | `sensor_msgs/msg/LaserScan` | SLAM/Nav2 obstacle source |
| `/perception/depth/obstacle_points` | `sensor_msgs/msg/PointCloud2` | local costmap의 낮은 장애물 source |
| `/perception/depth/clearing_points` | `sensor_msgs/msg/PointCloud2` | local costmap의 Depth free-space raytracing |
| `/map` | `nav_msgs/msg/OccupancyGrid` | SLAM 또는 map server 지도 |

## Behavior 수동 테스트

```bash
ros2 topic echo /planning/behavior_state

ros2 topic pub --times 3 /planning/behavior_cmd \
  std_msgs/msg/String "{data: stop}"

ros2 topic pub --times 3 /planning/behavior_cmd \
  std_msgs/msg/String "{data: run}"

ros2 topic echo /control/cmd_vel_safe
```

DDS discovery로 첫 메시지가 유실될 수 있어 테스트에서는 `--times 3`을 권장합니다.

## Launch 파일

| launch | 용도 |
|---|---|
| `behavior_nav2_demo.launch.py` | fake scan/odom을 사용한 RViz Nav2 데모 |
| `slam_bringup.launch.py` | 전방 LiDAR를 포함한 실제 mapping 진입점 |
| `mapping_navigation.launch.py` | motor + SLAM + Nav2; scan은 외부 제공 필요 |
| `map_navigation.launch.py` | 전방 LiDAR + 저장 지도 AMCL + Nav2 |
| `manual_slam_bringup.launch.py` | `/control/cmd_vel_test` 기반 수동 SLAM 시험 |
| `nav2_fake.launch.py` | Nav2 navigation include만 제공하는 저수준 launch |

## 설정 파일

- `helper_nav2_params.yaml`: 실제 로봇 Nav2/AMCL/costmap/velocity smoother
- `helper_nav2_fake_params.yaml`: demo용 Nav2 설정
- `helper_slam_params.yaml`: SLAM Toolbox와 scan topic 설정
- `maps/*.yaml`, `maps/*.pgm`: 저장 지도

실제 local costmap은 `scan depth_mark depth_clear`를 observation source로
사용합니다. `depth_mark`는 높이 0.10~0.30 m point를 marking하고,
`depth_clear`는 CameraInfo가 제공하는 optical frame의 유효 depth point로
0.2~2.0 m free space를 raytracing합니다.
0.04 m 이상 0.10 m 미만의 낮은 물체는 costmap에서 제외하고 perception이
`overcome`으로 처리합니다. 정적 장애물은 별도 `avoid` 명령 없이 `run`을
유지한 상태에서 Nav2가 회피합니다. global costmap은 현재 전방 LiDAR `scan`만
사용합니다.

## 점검 순서

```bash
ros2 topic echo /perception/scan/filtered --once
ros2 topic echo /control/odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_front
ros2 run tf2_ros tf2_echo map odom
ros2 action list | grep navigate_to_pose
```
