# helper_navigation

`helper_navigation`은 helper robot의 SLAM, map 기반 주행, Nav2 연동, behavior 제어 흐름을 담당하는
ROS 2 navigation 패키지입니다.

현재 패키지는 아래 세 가지 실행 흐름을 제공합니다.

```text
1. RViz demo 주행 검증
2. SLAM mapping
3. 저장된 map 기반 navigation
```

## 전체 구조

기본 주행 파이프라인은 아래와 같습니다.

```text
/planning/goal_pose
-> behavior_manager_node
-> Nav2 navigate_to_pose
-> /control/cmd_vel
-> velocity_smoother
-> /control/cmd_vel_smoothed
-> cmd_vel_safety_gate_node
-> /control/cmd_vel_safe
-> demo odom 또는 motor_driver_node
```

behavior 제어 흐름은 아래와 같습니다.

```text
perception / test / VDA5050 adapter
-> /planning/behavior_cmd
-> behavior_manager_node
-> /planning/behavior_state
-> cmd_vel_safety_gate_node
```

`/planning/behavior_cmd`는 외부에서 들어오는 명령이고,
`/planning/behavior_state`는 현재 behavior 상태를 확인하기 위한 topic입니다.

## 주요 노드

### behavior_manager_node

파일:

```text
scripts/behavior_manager_node.py
```

역할:

```text
/planning/goal_pose를 받아 Nav2 navigate_to_pose action으로 전달
/planning/behavior_cmd를 받아 stop/run/overcome/avoid 처리
/planning/behavior_state를 주기적으로 publish
```

지원 behavior:

| Behavior | 의미 |
|---|---|
| `run` | 정상 주행, stop으로 멈춘 goal 재개 |
| `stop` | Nav2 goal cancel, safety gate에서 최종 속도 0 차단 |
| `overcome` | 낮은 speed limit 적용 후 주행 |
| `avoid` | costmap clear 후 현재 goal 재시작 |

### demo_cmd_vel_odom_node

파일:

```text
scripts/demo_cmd_vel_odom_node.py
```

역할:

```text
/control/cmd_vel_safe
-> /control/odom
-> odom -> base_link TF
```

실제 모터 없이 RViz에서 로봇이 움직이는 것처럼 확인하기 위한 demo용 fake odom 노드입니다.
실제 로봇 주행에서는 사용하지 않습니다.

## Launch 파일

### behavior_nav2_demo.launch.py

파일:

```text
launch/behavior_nav2_demo.launch.py
```

RViz에서 behavior -> Nav2 -> cmd_vel gate 흐름을 확인하기 위한 demo launch입니다.

포함 구성:

```text
robot_state_publisher
cmd_vel_safety_gate_node
demo_cmd_vel_odom_node
fake_scan
Nav2 navigation_launch.py
behavior_manager_node
RViz optional
```

실행:

```bash
cd ~/workspace/helper_robotics
source install/setup.bash
ros2 launch helper_navigation behavior_nav2_demo.launch.py rviz:=true
```

이 launch는 실제 LiDAR와 실제 motor를 사용하지 않습니다.

### mapping_navigation.launch.py

파일:

```text
launch/mapping_navigation.launch.py
```

실제 로봇으로 SLAM mapping을 수행하기 위한 launch입니다.

포함 구성:

```text
robot_state_publisher
motor_driver.launch.py
slam_toolbox online_async_launch.py
Nav2 navigation_launch.py
behavior_manager_node
RViz optional
```

실행:

```bash
cd ~/workspace/helper_robotics
source install/setup.bash
ros2 launch helper_navigation mapping_navigation.launch.py rviz:=true motor:=true
```

모터 없이 먼저 확인할 경우:

```bash
ros2 launch helper_navigation mapping_navigation.launch.py rviz:=true motor:=false
```

이 launch는 실제 입력으로 아래 topic/TF가 필요합니다.

```text
/control/odom
/perception/scan/filtered
odom -> base_link
base_link -> laser_front / laser_rear
```

SLAM이 정상 동작하면 아래가 생성됩니다.

```text
/map
map -> odom
```

지도 저장:

```bash
mkdir -p ~/workspace/helper_robotics/src/helper_navigation/maps
ros2 run nav2_map_server map_saver_cli -f ~/workspace/helper_robotics/src/helper_navigation/maps/helper_map
```

저장 결과 예시:

```text
src/helper_navigation/maps/helper_map.yaml
src/helper_navigation/maps/helper_map.pgm
```

### map_navigation.launch.py

파일:

```text
launch/map_navigation.launch.py
```

SLAM으로 저장한 map을 불러와 주행하기 위한 launch입니다.

포함 구성:

```text
robot_state_publisher
motor_driver.launch.py
Nav2 localization_launch.py
Nav2 navigation_launch.py
behavior_manager_node
RViz optional
```

실행:

```bash
cd ~/workspace/helper_robotics
source install/setup.bash
ros2 launch helper_navigation map_navigation.launch.py \
  map:=/home/jiming/workspace/helper_robotics/src/helper_navigation/maps/helper_map.yaml \
  rviz:=true \
  motor:=true
```

모터 없이 map load/localization만 확인할 경우:

```bash
ros2 launch helper_navigation map_navigation.launch.py \
  map:=/home/jiming/workspace/helper_robotics/src/helper_navigation/maps/helper_map.yaml \
  rviz:=true \
  motor:=false
```

`map_navigation.launch.py`는 저장된 map을 사용하므로 `map:=...` 인자가 필요합니다.

## 주요 Topic

### Planning / Behavior

| Topic | Type | 설명 |
|---|---|---|
| `/planning/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz, VDA5050 adapter, ACS 변환 노드가 보내는 목표 위치 |
| `/planning/behavior_cmd` | `std_msgs/msg/String` | perception/test/VDA5050 adapter가 보내는 behavior 명령 |
| `/planning/behavior_state` | `std_msgs/msg/String` | 현재 behavior 상태. `run`, `stop`, `overcome`, `avoid` |
| `/speed_limit` | `nav2_msgs/msg/SpeedLimit` | `overcome` 상태에서 Nav2 속도 제한용 |

### Control

| Topic | Type | 설명 |
|---|---|---|
| `/control/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 controller 출력 |
| `/control/cmd_vel_smoothed` | `geometry_msgs/msg/Twist` | Nav2 velocity_smoother 출력 |
| `/control/cmd_vel_safe` | `geometry_msgs/msg/Twist` | safety gate를 거친 최종 속도 명령 |
| `/control/odom` | `nav_msgs/msg/Odometry` | 로봇 odom. demo에서는 fake, 실제에서는 encoder/odom 기반 |

### Sensor / Map

| Topic | Type | 설명 |
|---|---|---|
| `/perception/scan/filtered` | `sensor_msgs/msg/LaserScan` | Nav2/SLAM이 사용하는 최종 LiDAR scan |
| `/map` | `nav_msgs/msg/OccupancyGrid` | SLAM 또는 map_server가 publish하는 지도 |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Nav2, SLAM, RViz에서 사용하는 TF |

## Behavior 테스트 명령어

현재 상태 확인:

```bash
ros2 topic echo /planning/behavior_state
```

stop 명령:

```bash
ros2 topic pub --times 3 /planning/behavior_cmd std_msgs/msg/String "{data: stop}"
```

run 명령:

```bash
ros2 topic pub --times 3 /planning/behavior_cmd std_msgs/msg/String "{data: run}"
```

`--once`는 DDS discovery 타이밍 때문에 놓칠 수 있어 테스트 시 `--times 3`을 권장합니다.

최종 안전 속도 확인:

```bash
ros2 topic echo /control/cmd_vel_safe
```

## Nav2 설정 파일

### helper_nav2_params.yaml

파일:

```text
config/helper_nav2_params.yaml
```

실제 로봇용 Nav2 설정입니다.

주요 설정:

```text
AMCL
map_server
planner_server
controller_server
local_costmap
global_costmap
velocity_smoother
```

중요 topic/frame:

```text
scan_topic: /perception/scan/filtered
odom_topic: /control/odom
robot_base_frame: base_link
global_frame: map
```

### helper_nav2_fake_params.yaml

파일:

```text
config/helper_nav2_fake_params.yaml
```

RViz demo용 Nav2 설정입니다. 실제 map 없이 fake odom 기반으로 behavior/Nav2 흐름을 확인하기 위한 설정입니다.

## 실제 로봇 적용 시 필요한 것

실제 주행을 위해서는 아래 항목들이 준비되어야 합니다.

```text
1. 실제 /control/odom
2. 실제 /perception/scan/filtered
3. odom -> base_link TF
4. base_link -> laser_front / laser_rear TF
5. LiDAR 2개 scan merge/filter
6. motor_driver_node가 /control/cmd_vel_safe를 받아 실제 구동
7. SLAM map 저장 및 map 기반 localization 검증
```

LiDAR 2개는 추후 아래 흐름으로 구성하는 것을 목표로 합니다.

```text
front LiDAR
rear LiDAR
-> scan merge/filter
-> /perception/scan/filtered
```

## VDA5050 / ACS 연동 위치

VDA5050 adapter는 이 패키지의 내부 topic과 연결됩니다.

```text
ACS / VDA5050 order
-> helper_vda5050
-> /planning/goal_pose
-> behavior_manager_node
-> Nav2
```

```text
ACS / VDA5050 instantActions
-> helper_vda5050
-> /planning/behavior_cmd
```

따라서 ACS 연동은 Nav2를 직접 대체하는 것이 아니라, 상위 명령을 helper robot 내부 planning topic으로 변환하는 adapter 역할입니다.

## 현재 검증 상태

현재 확인된 범위:

```text
RViz demo에서 behavior -> Nav2 -> cmd_vel_safety_gate 흐름 확인
stop/run behavior command 확인
/control/cmd_vel_safe 차단 확인
mapping/map navigation launch 구조 분리
```

추가 검증 필요:

```text
실제 LiDAR scan
실제 odom
TF 전체 구조
SLAM map 저장
저장된 map 기반 localization
실제 motor 구동
ACS/VDA5050 실연동
```
