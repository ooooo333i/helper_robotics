# helper_vda5050

VDA5050 형식의 MQTT order/instant action을 ROS 2 navigation 명령으로 변환하고,
odometry와 behavior 상태를 MQTT state로 발행합니다. 전체 표준이 아닌 주행 데모용
최소 subset입니다.

## 준비 및 빌드

```bash
sudo apt install -y python3-paho-mqtt mosquitto mosquitto-clients
sudo systemctl enable --now mosquitto

cd ~/workspace/helper_robotics
colcon build --symlink-install --packages-select helper_vda5050
source install/setup.bash
```

## 실행

별도 터미널에서 navigation, adapter, panel을 실행합니다.

```bash
ros2 launch helper_navigation behavior_nav2_demo.launch.py rviz:=true
ros2 launch helper_vda5050 vda5050_adapter.launch.py broker_host:=localhost
ros2 launch helper_vda5050 vda5050_demo_panel.launch.py broker_host:=localhost
```

브라우저에서 <http://127.0.0.1:8088>을 열어 order, stop, resume,
`cancelOrder`를 테스트합니다.

## 흐름

```text
MQTT .../order -> /planning/goal_pose -> behavior_manager/Nav2
MQTT .../instantActions -> /planning/behavior_cmd
/control/odom + /planning/behavior_state -> MQTT .../state
```

기본 MQTT prefix는 `vda5050/v3/helper/helper_001`입니다.

```bash
mosquitto_sub -h localhost -t 'vda5050/v3/helper/helper_001/state'
ros2 topic echo /planning/goal_pose
ros2 topic echo /planning/behavior_state
```

identity와 broker는 launch 인자로 변경할 수 있습니다.

```bash
ros2 launch helper_vda5050 vda5050_adapter.launch.py \
  broker_host:=192.168.0.10 broker_port:=1883 \
  manufacturer:=helper serial_number:=helper_001
```
