# helper_vda5050

VDA5050 형식의 MQTT order/instant action을 ROS 2 navigation 명령으로 변환하고,
odometry와 behavior 상태를 MQTT state로 발행합니다.

## 준비 및 빌드

```bash
sudo apt install -y python3-paho-mqtt mosquitto mosquitto-clients
sudo systemctl enable --now mosquitto

cd ~/helper_robotics
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

## 지원 메시지

- `order`: 마지막 released node의 `x`, `y`, `theta`, `mapId`를 Nav2 목표점으로 변환
- `instantActions`: `stop`, `pause`, `cancelOrder`, `start`, `resume` 처리
- `state`: order ID, 위치·자세, driving, operating mode, behavior 및 safety 상태 발행

MQTT topic은 다음 규칙으로 생성됩니다.

```text
{interface_name}/{major_version}/{manufacturer}/{serial_number}/order
{interface_name}/{major_version}/{manufacturer}/{serial_number}/instantActions
{interface_name}/{major_version}/{manufacturer}/{serial_number}/state
```
