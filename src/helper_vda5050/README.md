# helper_vda5050

`helper_vda5050`은 VDA5050 형식의 MQTT order/instant action을 내부 ROS 2
navigation topic으로 변환하고, 로봇 pose와 behavior를 MQTT state로 내보내는
adapter입니다. 전체 VDA5050 표준 구현이 아니라 현재 주행 데모에 필요한 최소
subset입니다.

## 준비와 실행

```bash
sudo apt update
sudo apt install python3-paho-mqtt mosquitto mosquitto-clients

cd ~/workspace/helper_robotics
colcon build --symlink-install --packages-select \
  helper_vda5050 helper_navigation
source install/setup.bash
```

broker와 adapter를 실행합니다.

```bash
sudo systemctl enable --now mosquitto
ros2 launch helper_vda5050 vda5050_adapter.launch.py \
  broker_host:=localhost \
  broker_port:=1883 \
  manufacturer:=helper \
  serial_number:=helper_001
```

로컬 웹 테스트 panel은 별도 터미널에서 실행합니다.

```bash
ros2 launch helper_vda5050 vda5050_demo_panel.launch.py
```

브라우저에서 `http://127.0.0.1:8088`을 열면 order, stop, resume, pause,
cancelOrder를 시험할 수 있습니다.

## MQTT 입출력 형식

기본 topic prefix는 다음과 같습니다.

```text
vda5050/v3/helper/helper_001
```

| 구분 | MQTT topic | payload |
|---|---|---|
| 입력 | `.../order` | JSON object; `orderId`, `orderUpdateId`, `nodes[]` |
| 입력 | `.../instantActions` | JSON object; `instantActions[]` 또는 `actions[]` |
| 출력 | `.../state` | JSON VDA5050-like state, 기본 2 Hz |

order에서는 `released=true`이고 `nodePosition`이 있는 마지막 node를 선택합니다.
없으면 `nodePosition`이 있는 마지막 node를 선택합니다. 사용 필드는
`x`, `y`, `theta` [rad], `mapId`입니다.

instant action mapping:

| `actionType` | ROS behavior |
|---|---|
| `stop`, `pause`, `cancelOrder` | `stop` |
| `start`, `resume` | `run` |
| 그 외 | 로그 후 무시 |

state에는 마지막 order id, odometry pose, behavior 정보, 간단한
`batteryState`, `safetyState`, `driving`이 포함됩니다. 배터리 값 등은 실제
장치 상태가 아닌 초기값입니다.

## ROS 2 입출력

| 구분 | topic | 타입 | 내용 |
|---|---|---|---|
| 출력 | `/planning/goal_pose` | `geometry_msgs/msg/PoseStamped` | order의 마지막 목표 |
| 출력 | `/planning/behavior_cmd` | `std_msgs/msg/String` | instant action의 `run/stop` |
| 입력 | `/planning/behavior_state` | `std_msgs/msg/String` | MQTT state에 포함 |
| 입력 | `/control/odom` | `nav_msgs/msg/Odometry` | MQTT `agvPosition`에 포함 |

## 구현 흐름

```text
MQTT order JSON
  -> 마지막 target node 선택
  -> theta를 quaternion으로 변환
  -> /planning/goal_pose
  -> behavior_manager -> Nav2

MQTT instantActions
  -> actionType mapping
  -> /planning/behavior_cmd

/control/odom + /planning/behavior_state
  -> state JSON
  -> MQTT .../state
```

MQTT network loop는 별도 thread에서 실행되며 ROS timer가 state를 발행합니다.
QoS는 MQTT와 ROS publisher 모두 기본적인 best-effort 목적의 소규모 데모
구성입니다.

## 전체 데모

터미널 1:

```bash
ros2 launch helper_navigation behavior_nav2_demo.launch.py rviz:=true
```

터미널 2:

```bash
ros2 launch helper_vda5050 vda5050_adapter.launch.py
```

터미널 3:

```bash
ros2 launch helper_vda5050 vda5050_demo_panel.launch.py
```

터미널 4:

```bash
ros2 topic echo /planning/goal_pose
ros2 topic echo /planning/behavior_state
mosquitto_sub -h localhost -t vda5050/v3/helper/helper_001/state
```

## CLI로 order 발행

```bash
mosquitto_pub -h localhost \
  -t vda5050/v3/helper/helper_001/order \
  -m '{
    "orderId":"test_001",
    "orderUpdateId":0,
    "nodes":[
      {
        "nodeId":"goal",
        "sequenceId":0,
        "released":true,
        "nodePosition":{"x":1.0,"y":0.0,"theta":0.0,"mapId":"map"}
      }
    ],
    "edges":[]
  }'
```

stop 예시:

```bash
mosquitto_pub -h localhost \
  -t vda5050/v3/helper/helper_001/instantActions \
  -m '{
    "instantActions":[
      {"actionId":"stop_001","actionType":"stop","blockingType":"HARD"}
    ]
  }'
```

## 주요 파라미터

`broker_host`, `broker_port`, `interface_name`, `major_version`,
`manufacturer`, `serial_number`, `map_frame`, `state_publish_rate_hz`로 MQTT
주소와 identity를 바꿀 수 있습니다. demo panel은 추가로 `http_host`,
`http_port`, `default_map_id`를 사용합니다.
