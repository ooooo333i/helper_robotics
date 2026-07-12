# Helper Robotics

실내 자율주행 로봇용 ROS 2 워크스페이스입니다. LiDAR/depth 장애물 인식,
SLAM·Nav2 주행, 속도 안전 제어, MD200T 모터 구동, VDA5050 MQTT 연동을
포함합니다.

- 기준 환경: Ubuntu 22.04, ROS 2 Humble, Python 3.10
- 패키지별 상세 실행법: 각 `src/<package>/README.md` 참고

## 전체 흐름

```text
LiDAR / Depth Camera
        ↓
helper_perception ──→ obstacle / behavior command
        ↓                         ↓
SLAM 또는 AMCL + Nav2 ──→ helper_navigation
                                  ↓ cmd_vel
                          helper_control safety gate
                                  ↓
                            MD200T motor driver

VDA5050 MQTT order ──→ helper_vda5050 ──→ navigation goal
```

## 설치 및 빌드

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool

cd ~/workspace/helper_robotics
vcs import src < dependencies.repos
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install
source install/setup.bash
```

새 터미널마다 다음 환경을 불러옵니다.

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/helper_robotics/install/setup.bash
```

## 실행

### 장치 없는 Nav2 데모

```bash
ros2 launch helper_navigation behavior_nav2_demo.launch.py rviz:=true
```

RViz에서 `2D Goal Pose`로 목표를 지정합니다.

### 실제 로봇 SLAM

처음에는 바퀴를 띄우고 비상 정지 수단을 준비하십시오.

```bash
cd ~/workspace/helper_robotics
./scripts/usb_port_setup.sh scan
./scripts/usb_port_setup.sh configure
source config/usb_ports.env
./scripts/usb_port_setup.sh check

ros2 launch helper_navigation slam_bringup.launch.py motor:=true rviz:=true
```

지도 저장:

```bash
ros2 run nav2_map_server map_saver_cli -f \
  ~/workspace/helper_robotics/src/helper_navigation/maps/helper_map
```

### 저장 지도 자율주행

```bash
source ~/workspace/helper_robotics/config/usb_ports.env
ros2 launch helper_navigation map_navigation.launch.py \
  map:=$HOME/workspace/helper_robotics/src/helper_navigation/maps/helper_map.yaml \
  motor:=true rviz:=true
```

RViz에서 `2D Pose Estimate`로 초기 위치를 설정하고 `2D Goal Pose`를 보냅니다.
Depth 장애물 인식을 함께 사용할 때는 별도 터미널에서 실행합니다.

```bash
ros2 launch helper_perception depth_obstacle.launch.py
ros2 launch helper_perception perception_behavior_gate.launch.py
```

### VDA5050 가상 데모

```bash
sudo apt install -y mosquitto mosquitto-clients python3-paho-mqtt
sudo systemctl enable --now mosquitto
```

각 명령을 별도 터미널에서 실행합니다.

```bash
ros2 launch helper_navigation behavior_nav2_demo.launch.py rviz:=true
ros2 launch helper_vda5050 vda5050_adapter.launch.py broker_host:=localhost
ros2 launch helper_vda5050 vda5050_demo_panel.launch.py broker_host:=localhost
```

브라우저에서 <http://127.0.0.1:8088>을 엽니다.

## 패키지

| 패키지 | 역할 |
|---|---|
| `helper_msgs` | 공통 custom message |
| `helper_description` | URDF와 센서 TF |
| `helper_perception` | LiDAR/depth 처리와 장애물 판단 |
| `helper_navigation` | SLAM, AMCL, Nav2, behavior 관리 |
| `helper_control` | 속도 안전 처리와 모터 구동 |
| `helper_status` | 로봇 상태 발행 |
| `helper_vda5050` | MQTT VDA5050 adapter와 demo panel |
| `helper_cmd_vel_test` | 모터·장애물 제어 테스트 명령 |
