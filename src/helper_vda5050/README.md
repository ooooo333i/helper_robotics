# helper_vda5050

`helper_vda5050`는 ACS 관제 시스템과 helper robot 내부 ROS 2 navigation stack을 연결하기 위한
VDA5050 MQTT adapter 패키지입니다.

현재 구현은 VDA5050 전체 스펙을 모두 지원하는 완성형 구현이 아니라, **기본적인 네비게이션 연동을
검증하기 위한 skeleton**입니다.

## 역할

ACS에서 들어오는 VDA5050 MQTT 메시지를 ROS 2 내부 topic으로 변환합니다.

```text
ACS / VDA5050 MQTT
-> helper_vda5050 adapter
-> /planning/goal_pose
-> behavior_manager_node
-> Nav2
-> cmd_vel_safety_gate_node
-> motor_driver_node
```

로봇 상태는 반대 방향으로 ACS에 보고합니다.

```text
/control/odom + /planning/behavior_state
-> helper_vda5050 adapter
-> VDA5050 state MQTT
-> ACS
```

## 현재 구현 범위

현재 구현된 기능은 아래와 같습니다.

```text
VDA5050 order
-> /planning/goal_pose
```

```text
VDA5050 instantActions stop / pause / cancelOrder
-> /planning/behavior_cmd: stop
```

```text
VDA5050 instantActions start / resume
-> /planning/behavior_cmd: run
```

```text
/control/odom + /planning/behavior_state
-> VDA5050 state
```

## 기본 설정값

현재는 실제 ACS 설정값이 아직 확정되지 않았기 때문에 아래 기본값으로 동작합니다.
실제 ACS 연동 시 launch parameter로 변경할 수 있습니다.

| Parameter | Default | 설명 |
|---|---:|---|
| `broker_host` | `localhost` | MQTT broker 주소 |
| `broker_port` | `1883` | MQTT broker port |
| `interface_name` | `vda5050` | MQTT topic prefix |
| `major_version` | `v3` | VDA5050 major version topic level |
| `manufacturer` | `helper` | 제조사 topic level |
| `serial_number` | `helper_001` | 로봇 serial number topic level |
| `goal_topic` | `/planning/goal_pose` | VDA5050 order를 변환해 publish할 ROS goal topic |
| `behavior_cmd_topic` | `/planning/behavior_cmd` | instantActions를 변환해 publish할 behavior 명령 topic |
| `behavior_state_topic` | `/planning/behavior_state` | 로봇 내부 behavior 상태 topic |
| `odom_topic` | `/control/odom` | 로봇 odometry topic |
| `map_frame` | `map` | VDA5050 state 위치 보고에 사용할 map frame |
| `state_publish_rate_hz` | `2.0` | VDA5050 state publish 주기 |

기본 MQTT topic은 아래와 같습니다.

```text
vda5050/v3/helper/helper_001/order
vda5050/v3/helper/helper_001/instantActions
vda5050/v3/helper/helper_001/state
```

일반적인 topic 구조는 아래와 같습니다.

```text
{interface_name}/{major_version}/{manufacturer}/{serial_number}/{topic}
```

## 설치 항목

MQTT Python client가 필요합니다.

```bash
sudo apt update
sudo apt install python3-paho-mqtt
```

로컬 테스트를 위해 MQTT broker/client도 설치합니다.

```bash
sudo apt install mosquitto mosquitto-clients
```

설치 확인:

```bash
mosquitto -v
```

## 빌드

```bash
cd ~/workspace/helper_robotics
colcon build --symlink-install --packages-select helper_vda5050
source install/setup.bash
```

전체 workspace를 빌드하는 경우:

```bash
cd ~/workspace/helper_robotics
colcon build --symlink-install
source install/setup.bash
```

## 실행

로컬 MQTT broker 기준 실행:

```bash
ros2 launch helper_vda5050 vda5050_adapter.launch.py
```

ACS broker 정보가 있는 경우:

```bash
ros2 launch helper_vda5050 vda5050_adapter.launch.py \
  broker_host:=192.168.0.10 \
  broker_port:=1883 \
  manufacturer:=helper \
  serial_number:=helper_001
```

## Demo 기반 검증 방법

실제 ACS가 없어도 local MQTT broker와 RViz demo 환경을 이용해 기본 동작을 확인할 수 있습니다.

### 1. MQTT broker 실행

터미널 1:

```bash
mosquitto -v
```

### 2. Navigation demo 실행

터미널 2:

```bash
cd ~/workspace/helper_robotics
source install/setup.bash
ros2 launch helper_navigation behavior_nav2_demo.launch.py rviz:=true
```

### 3. VDA5050 adapter 실행

터미널 3:

```bash
cd ~/workspace/helper_robotics
source install/setup.bash
ros2 launch helper_vda5050 vda5050_adapter.launch.py
```

### 4. ROS topic 확인

터미널 4:

```bash
ros2 topic echo /planning/goal_pose
```

```bash
ros2 topic echo /planning/behavior_state
```

```bash
ros2 topic echo /control/cmd_vel_safe
```

VDA5050 state MQTT publish 확인:

```bash
mosquitto_sub -h localhost -t vda5050/v3/helper/helper_001/state
```

## Order 테스트

아래 MQTT message를 publish하면 adapter가 VDA5050 order를 `/planning/goal_pose`로 변환합니다.

```bash
mosquitto_pub -h localhost -t vda5050/v3/helper/helper_001/order -m '{
  "orderId": "test_order_001",
  "orderUpdateId": 0,
  "nodes": [
    {
      "nodeId": "start",
      "sequenceId": 0,
      "released": true,
      "nodePosition": {
        "x": 0.0,
        "y": 0.0,
        "theta": 0.0,
        "mapId": "odom"
      }
    },
    {
      "nodeId": "goal",
      "sequenceId": 2,
      "released": true,
      "nodePosition": {
        "x": 1.0,
        "y": 0.0,
        "theta": 0.0,
        "mapId": "odom"
      }
    }
  ],
  "edges": [
    {
      "edgeId": "edge_1",
      "sequenceId": 1,
      "startNodeId": "start",
      "endNodeId": "goal",
      "released": true
    }
  ]
}'
```

정상 동작 시:

```text
/planning/goal_pose 발행
RViz demo에서 goal 방향으로 주행
```

데모 환경은 `odom` frame 기준으로 동작하므로 테스트 order의 `mapId`를 `odom`으로 넣었습니다.
실제 map 기반 주행에서는 ACS 좌표계와 ROS `map` frame의 관계를 확인해야 합니다.

## Stop 테스트

아래 MQTT message를 publish하면 adapter가 VDA5050 instantActions를 `/planning/behavior_cmd: stop`으로 변환합니다.

```bash
mosquitto_pub -h localhost -t vda5050/v3/helper/helper_001/instantActions -m '{
  "headerId": 1,
  "timestamp": "2026-05-28T00:00:00Z",
  "version": "3.0.0",
  "manufacturer": "helper",
  "serialNumber": "helper_001",
  "instantActions": [
    {
      "actionId": "stop_001",
      "actionType": "stop",
      "blockingType": "HARD"
    }
  ]
}'
```

정상 동작 시:

```text
/planning/behavior_cmd: stop
/planning/behavior_state: stop
/control/cmd_vel_safe: 0
RViz demo 정지
```

## Resume 테스트

```bash
mosquitto_pub -h localhost -t vda5050/v3/helper/helper_001/instantActions -m '{
  "headerId": 2,
  "timestamp": "2026-05-28T00:00:01Z",
  "version": "3.0.0",
  "manufacturer": "helper",
  "serialNumber": "helper_001",
  "instantActions": [
    {
      "actionId": "resume_001",
      "actionType": "resume",
      "blockingType": "NONE"
    }
  ]
}'
```

정상 동작 시:

```text
/planning/behavior_state: run
정지했던 goal 재개
```

## 내부 ROS topic

이 adapter는 아래 ROS topic과 연결됩니다.

| Topic | Direction | 설명 |
|---|---|---|
| `/planning/goal_pose` | publish | VDA5050 order에서 변환된 목표 위치 |
| `/planning/behavior_cmd` | publish | VDA5050 instantActions에서 변환된 behavior 명령 |
| `/planning/behavior_state` | subscribe | 현재 behavior 상태 |
| `/control/odom` | subscribe | VDA5050 state 위치 보고에 사용할 odom |

## 실제 ACS 연동 전 확인 필요값

현재는 기본값으로 개발하고 있으며, 실제 ACS 연결 시 아래 값들은 반드시 맞춰야 합니다.

```text
VDA5050 version
MQTT broker host / port
TLS / username / password 필요 여부
manufacturer
serialNumber
topic prefix 규칙
ACS 좌표계와 ROS map 좌표계 일치 여부
좌표 단위 meter/radian 여부
theta 기준
필수 instantActions 목록
state 필수 필드
factsheet 요구 여부
```

## 현재 한계

현재 구현은 VDA5050 기본 navigation 연동 skeleton입니다.

구현됨:

```text
order -> /planning/goal_pose
instantActions -> /planning/behavior_cmd
/control/odom + /planning/behavior_state -> state
```

아직 미구현:

```text
JSON schema validation
TLS / 인증
factsheet
connection topic
full node/edge traversal state
action state lifecycle
map / zone management
VDA5050 error model
```

## 요약

현재 상태에서는 실제 ACS 없이도 local MQTT broker와 RViz demo를 이용해
VDA5050 `order` / `instantActions`가 내부 navigation pipeline으로 연결되는지 확인할 수 있습니다.

실제 ACS 연동 시에는 broker 정보, topic level, 좌표계, 필수 state field 등을 사측 설정에 맞춰 조정해야 합니다.
