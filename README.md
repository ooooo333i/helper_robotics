# helper_robotics

Helper robot의 센서 인식, Nav2/SLAM, behavior 제어, MD200T 모터 구동,
VDA5050 MQTT 연동을 하나의 ROS 2 workspace로 구성한 프로젝트입니다.

현재 기준 환경은 Ubuntu 22.04 + ROS 2 Humble입니다.

## 전체 구조

| 패키지 | 역할 | 상세 문서 |
|---|---|---|
| `helper_msgs` | 장애물·로봇 상태·리프트 custom message | [README](src/helper_msgs/README.md) |
| `helper_description` | 차체/바퀴/센서 URDF와 static TF | [README](src/helper_description/README.md) |
| `helper_perception` | LiDAR/depth 가공, 장애물 및 behavior 판단 | [README](src/helper_perception/README.md) |
| `helper_navigation` | SLAM, AMCL, Nav2, behavior manager | [README](src/helper_navigation/README.md) |
| `helper_control` | 속도 safety gate, MD200T, open-loop odometry | [README](src/helper_control/README.md) |
| `helper_status` | odometry 기반 robot status | [README](src/helper_status/README.md) |
| `helper_vda5050` | MQTT VDA5050 subset ↔ ROS 2 adapter | [README](src/helper_vda5050/README.md) |
| `helper_cmd_vel_test` | 모터/장애물 입력 시험 publisher | [README](src/helper_test/helper_cmd_vel_test/README.md) |

전체 동작 흐름은 다음과 같습니다.

```text
                    MQTT order / instantActions
                              │
                       helper_vda5050
                              │
LiDAR ─┐            /planning/goal_pose
Depth ─┼─> helper_perception ─> /planning/behavior_cmd
Path  ─┘                         │
                                v
                         behavior_manager
                     ┌──────────┴──────────┐
                     │ NavigateToPose      │ behavior_state
                     v                     v
                    Nav2            cmd_vel_safety_gate
                     │                     ^
             /control/cmd_vel              │
                     │                     │
              velocity_smoother            │
                     └─> cmd_vel_smoothed ─┘
                                           │
                                    cmd_vel_safe
                                           v
                               motor_driver -> MD200T
                                           │
                                /control/odom + TF
```

Depth는 behavior 판단과 별개로 낮은 장애물 point cloud를 local costmap에도
전달합니다.

```text
Depth Image + CameraInfo
  -> depth_obstacle_cloud_node
       ├─ /perception/depth/obstacle_points (marking PointCloud2)
       └─ /perception/depth/clearing_points (clearing PointCloud2)
  -> Nav2 local costmap VoxelLayer
```

TF는 아래 주체가 나누어 만듭니다.

```text
map -> odom                  SLAM Toolbox 또는 AMCL
odom -> base_link           motor_driver (현재 명령 기반 open-loop)
base_link -> laser/camera   robot_state_publisher + URDF
```

## 1. 저장소 clone

ROS 2 Humble이 설치된 Ubuntu 22.04에서 다음을 실행합니다.

```bash
mkdir -p ~/workspace
cd ~/workspace
git clone https://github.com/ooooo333i/helper_robotics.git
cd helper_robotics
```

특정 branch가 필요하면 clone 뒤 확인하고 전환합니다.

```bash
git branch -a
git switch <branch-name>
```

## 2. 의존성 설치

ROS 2와 개발 도구가 아직 없다면 먼저 ROS 2 Humble 공식 설치를 완료한 뒤
다음을 설치합니다.

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-numpy \
  python3-serial \
  python3-paho-mqtt \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-xacro \
  ros-humble-depthimage-to-laserscan
```

Intel RealSense를 depth camera로 쓸 경우 추가합니다.

```bash
sudo apt install -y ros-humble-realsense2-camera
```

LiDAR launch가 요구하는 `sllidar_ros2`는 이 저장소에 포함되어 있지 않습니다.
SLAMTEC의 ROS 2 driver를 같은 workspace에 clone합니다.

```bash
cd ~/workspace/helper_robotics/src
git clone https://github.com/Slamtec/sllidar_ros2.git
cd ..
```

이 driver는 SLAMTEC 공식 저장소의 설치 방식과 동일합니다:
[Slamtec/sllidar_ros2](https://github.com/Slamtec/sllidar_ros2).

나머지 package dependency를 설치합니다.

```bash
source /opt/ros/humble/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## 3. 빌드

```bash
cd ~/workspace/helper_robotics
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

새 터미널마다 ROS와 workspace를 source해야 합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/helper_robotics/install/setup.bash
```

편의를 위해 `~/.bashrc`에 추가할 수도 있지만, 여러 ROS workspace를 함께 쓰는
환경에서는 source 순서가 달라질 수 있으므로 현재 터미널에서 먼저 검증하십시오.

## 4. USB 장치 설정

가능하면 바뀌기 쉬운 `/dev/ttyUSB0` 대신 `/dev/serial/by-id/...` 경로를
사용합니다.

연결 장치 검색:

```bash
cd ~/workspace/helper_robotics
./scripts/usb_port_setup.sh scan
```

전방/후방 LiDAR, motor driver, depth camera가 모두 연결되어 있다면 대화형으로
환경 파일을 만들 수 있습니다.

```bash
./scripts/usb_port_setup.sh configure
source config/usb_ports.env
./scripts/usb_port_setup.sh check
```

일부 장치만 쓰는 경우에는 예제를 복사해 사용하는 값만 실제 경로로 수정합니다.

```bash
cp config/usb_ports.env.example config/usb_ports.env
${EDITOR:-nano} config/usb_ports.env
source config/usb_ports.env
```

serial 권한 오류가 나면 현재 사용자를 `dialout` group에 추가한 뒤 다시
로그인합니다.

```bash
sudo usermod -aG dialout "$USER"
```

## 5. 가장 먼저 해볼 데모

실제 장치 없이 전체 Nav2 → safety gate → odometry 흐름을 확인합니다.

```bash
cd ~/workspace/helper_robotics
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch helper_navigation behavior_nav2_demo.launch.py rviz:=true
```

RViz에서 `2D Goal Pose`로 목표를 지정합니다. 다른 터미널에서 정지/재개를
시험합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/helper_robotics/install/setup.bash

ros2 topic pub --times 3 /planning/behavior_cmd \
  std_msgs/msg/String "{data: stop}"

ros2 topic pub --times 3 /planning/behavior_cmd \
  std_msgs/msg/String "{data: run}"
```

## 6. 실제 로봇에서 SLAM

먼저 포트와 모터 방향을 확인하고, 로봇을 띄우거나 저속 시험을 거친 뒤 바닥에서
실행하십시오.

```bash
cd ~/workspace/helper_robotics
source /opt/ros/humble/setup.bash
source install/setup.bash
source config/usb_ports.env

ros2 launch helper_navigation slam_bringup.launch.py \
  motor:=true \
  rviz:=true
```

실제 모터 없이 scan/SLAM만 확인하려면:

```bash
ros2 launch helper_navigation slam_bringup.launch.py \
  motor:=false \
  rviz:=true
```

정상 조건:

```bash
ros2 topic echo /perception/scan/filtered --once
ros2 topic echo /control/odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_front
ros2 run tf2_ros tf2_echo map odom
```

지도 저장:

```bash
ros2 run nav2_map_server map_saver_cli -f \
  ~/workspace/helper_robotics/src/helper_navigation/maps/helper_map
```

`helper_map.yaml`과 `helper_map.pgm`이 생성됩니다. 새 map을 git에 포함할지는
팀 정책에 맞게 결정합니다.

## 7. 저장 지도에서 최종 주행

### 필수 bringup

`map_navigation.launch.py` 한 번으로 전방 LiDAR, URDF, motor/safety gate,
map server/AMCL, Nav2, behavior manager를 실행합니다.

```bash
cd ~/workspace/helper_robotics
source /opt/ros/humble/setup.bash
source install/setup.bash
source config/usb_ports.env

ros2 launch helper_navigation map_navigation.launch.py \
  map:=$HOME/workspace/helper_robotics/src/helper_navigation/maps/helper_map.yaml \
  front_lidar_port:=${AMR_FRONT_LIDAR_PORT} \
  motor_port:=${AMR_MOTOR_DRIVER_PORT} \
  motor:=true \
  rviz:=true
```

RViz에서 먼저 `2D Pose Estimate`로 초기 위치를 지정하고 `2D Goal Pose`로 목표를
보냅니다.

### Depth와 자동 behavior를 함께 쓸 때

Intel RealSense 예시는 터미널을 나눠 실행합니다.

터미널 1:

```bash
ros2 launch realsense2_camera rs_launch.py enable_depth:=true
```

터미널 2:

```bash
ros2 launch helper_perception depth_obstacle.launch.py
```

터미널 3:

```bash
ros2 launch helper_perception perception_behavior_gate.launch.py
```

현재 behavior gate 설정은 Nav2 local path를 `/local_plan`에서 받습니다.
아래 명령으로 실제 발행 여부를 확인하십시오. 없다면 Nav2 publisher 설정 또는
`config/behavior_gate.yaml`의 `path_topic`을 실제 topic에 맞춰야 합니다.

```bash
ros2 topic info /local_plan -v
```

`depth_obstacle.launch.py`는 장애물 판단과 함께 marking/clearing PointCloud2를
발행합니다. 실제 Nav2 local costmap은 두 topic을 각각 `depth_mark`,
`depth_clear` source로 사용하므로 다음을 확인합니다.

```bash
ros2 topic echo /perception/depth/obstacle_points --once
ros2 topic echo /perception/depth/clearing_points --once
ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame
```

### VDA5050 관제 입력을 함께 쓸 때

```bash
sudo systemctl enable --now mosquitto
ros2 launch helper_vda5050 vda5050_adapter.launch.py \
  broker_host:=localhost
```

MQTT order가 `/planning/goal_pose`, instant action이
`/planning/behavior_cmd`로 변환됩니다.

## 주요 입출력

| topic | 타입 | 생산자 → 소비자 |
|---|---|---|
| `/perception/scan/filtered` | `sensor_msgs/msg/LaserScan` | LiDAR filter → SLAM/Nav2/behavior gate |
| `/perception/obstacle/depth` | `helper_msgs/msg/ObstacleDecision` | depth detector → fusion/behavior gate |
| `/perception/depth/obstacle_points` | `sensor_msgs/msg/PointCloud2` | depth cloud → Nav2 local costmap |
| `/perception/depth/clearing_points` | `sensor_msgs/msg/PointCloud2` | valid depth → local costmap clearing |
| `/planning/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz/VDA5050 → behavior manager |
| `/planning/behavior_cmd` | `std_msgs/msg/String` | perception/VDA5050 → behavior manager |
| `/planning/behavior_state` | `std_msgs/msg/String` | behavior manager → safety gate/VDA5050 |
| `/control/cmd_vel_smoothed` | `geometry_msgs/msg/Twist` | Nav2 smoother → safety gate |
| `/control/cmd_vel_safe` | `geometry_msgs/msg/Twist` | safety gate → motor driver |
| `/control/odom` | `nav_msgs/msg/Odometry` | motor driver → Nav2/status/VDA5050 |

각 custom message의 정확한 필드는
[`helper_msgs` 문서](src/helper_msgs/README.md)를 참고하십시오.

## 현재 구현에서 알아둘 점

- motor odometry는 엔코더가 아니라 명령 RPM 적분값이므로 장거리에서 오차가
  누적됩니다.
- `helper_status`의 theta, 속도, 배터리, action 일부는 아직 고정값입니다.
- `helper_vda5050`은 전체 표준이 아닌 order/instant action/state 최소 subset입니다.
- behavior manager의 현재 `avoid`는 costmap clear/replan 함수를 호출하지 않고
  현재 goal을 유지합니다. 회피와 주기적 경로 재생성은 Nav2의 DWB와 기본 BT가
  담당합니다.
- depth camera driver는 helper launch에 포함되지 않아 별도로 실행해야 합니다.
- `perception_behavior_gate`는 fresh local path가 없으면 behavior를 발행하지 않습니다.
- Depth cloud는 local costmap에 활성화되어 있지만 현재 ROI/거리/높이/sampling
  필터만 있고 cluster·여러 frame 확인·confidence 로직은 없습니다.
- Depth marking cloud는 높이 0.05~0.30 m point를 등록하고, 별도 clearing
  cloud는 카메라 optical frame에서 free space를 raytracing합니다.
  0.02 m 이상 0.05 m 미만은 costmap에서 제외하고 `overcome`으로 처리합니다.
  정적 장애물은 behavior `run`을 유지하고 Nav2 costmap 회피에 맡깁니다.
  camera TF, 높이/pitch, false marking, 장애물 제거 후 clearing을 RViz에서
  검증한 뒤 실주행해야 합니다.

## 테스트

```bash
cd ~/workspace/helper_robotics
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon test
colcon test-result --verbose
```

모터 자체 테스트는 안전을 확보한 뒤
[`helper_cmd_vel_test` 문서](src/helper_test/helper_cmd_vel_test/README.md)를
따르십시오.
