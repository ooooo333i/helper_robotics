# helper_vda5050 테스트 방법

`helper_vda5050`는 VDA5050 MQTT 메시지를 helper robot 내부 ROS 2 topic으로 변환하는 adapter입니다.

이 문서는 두 가지 테스트 방법만 정리합니다.

```text
1. RViz + Demo Panel로 버튼 테스트
2. mosquitto 명령어로 직접 MQTT publish 테스트
```

## 사전 준비

필요 패키지:

```bash
sudo apt update
sudo apt install python3-paho-mqtt mosquitto mosquitto-clients
```

빌드:

```bash
cd ~/workspace/helper_robotics
colcon build --symlink-install --packages-select helper_vda5050 helper_navigation
source install/setup.bash
```

MQTT broker는 `localhost:1883` 기준입니다.

`mosquitto -v` 실행 시 아래 메시지가 나오면 이미 broker가 실행 중인 상태입니다.

```text
Error: Address already in use
```

이 경우 broker를 새로 켜지 말고 바로 테스트를 진행하면 됩니다.

## 1. RViz + Demo Panel 테스트

### 터미널 1: Navigation demo + RViz 실행

```bash
cd ~/workspace/helper_robotics
source install/setup.bash
ros2 launch helper_navigation behavior_nav2_demo.launch.py rviz:=true
```

### 터미널 2: VDA5050 adapter 실행

```bash
cd ~/workspace/helper_robotics
source install/setup.bash
ros2 launch helper_vda5050 vda5050_adapter.launch.py
```

### 터미널 3: VDA5050 demo panel 실행

```bash
cd ~/workspace/helper_robotics
source install/setup.bash
ros2 launch helper_vda5050 vda5050_demo_panel.launch.py
```

브라우저에서 아래 주소를 엽니다.

```text
http://127.0.0.1:8088
```

Demo panel에서 아래 버튼으로 테스트할 수 있습니다.

```text
Send Order
Stop
Resume
Pause
Cancel Order
```

### 터미널 4: 상태 확인

behavior 상태:

```bash
ros2 topic echo /planning/behavior_state
```

goal 변환 확인:

```bash
ros2 topic echo /planning/goal_pose
```

최종 안전 속도 확인:

```bash
ros2 topic echo /control/cmd_vel_safe
```

VDA5050 state MQTT 확인:

```bash
mosquitto_sub -h localhost -t vda5050/v3/helper/helper_001/state
```

### 테스트 순서

```text
1. RViz가 뜨는지 확인
2. adapter가 MQTT connected 되는지 확인
3. demo panel 접속
4. Send Order 클릭
5. RViz에서 로봇 이동 확인
6. Stop 클릭
7. /planning/behavior_state가 stop인지 확인
8. /control/cmd_vel_safe가 0인지 확인
9. Resume 클릭
10. /planning/behavior_state가 run으로 바뀌는지 확인
```

## 2. mosquitto 명령어 직접 테스트

Demo panel 없이 MQTT 명령어를 직접 publish해서 테스트할 수도 있습니다.

### 터미널 1: Navigation demo + RViz 실행

```bash
cd ~/workspace/helper_robotics
source install/setup.bash
ros2 launch helper_navigation behavior_nav2_demo.launch.py rviz:=true
```

### 터미널 2: VDA5050 adapter 실행

```bash
cd ~/workspace/helper_robotics
source install/setup.bash
ros2 launch helper_vda5050 vda5050_adapter.launch.py
```

### 터미널 3: 상태 확인

```bash
ros2 topic echo /planning/behavior_state
```

```bash
ros2 topic echo /planning/goal_pose
```

```bash
ros2 topic echo /control/cmd_vel_safe
```

### Order publish

아래 명령어는 VDA5050 `order` 메시지를 publish합니다.
adapter가 이를 받아 `/planning/goal_pose`로 변환합니다.

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

정상 동작:

```text
/planning/goal_pose 발행
RViz demo 로봇 이동
```

### Stop publish

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

정상 동작:

```text
/planning/behavior_state: stop
/control/cmd_vel_safe: 0
RViz demo 로봇 정지
```

### Resume publish

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

정상 동작:

```text
/planning/behavior_state: run
정지했던 goal 재개
```

### VDA5050 state 확인

adapter가 publish하는 VDA5050 state는 아래 명령어로 확인합니다.

```bash
mosquitto_sub -h localhost -t vda5050/v3/helper/helper_001/state
```
