# helper_cmd_vel_test

`/control/cmd_vel_safe` 모터 수신 테스트용 ROS2 패키지입니다.

## Build

```bash
colcon build --packages-select helper_cmd_vel_test
source install/setup.bash
```

## Topic 수신 확인

기본 실행은 속도 0으로 publish합니다.

```bash
ros2 run helper_cmd_vel_test cmd_vel_test
```

다른 터미널에서 확인합니다.

```bash
ros2 topic echo /control/cmd_vel_safe
```

## 전진 테스트

```bash
ros2 run helper_cmd_vel_test cmd_vel_test --ros-args \
  -p linear_x:=0.1 \
  -p angular_z:=0.0
```

## 50cm 전진 테스트

기본값은 `linear_x=0.1`로 5초 동안 publish해서 50cm 전진 명령을 보낸 뒤
정지 명령을 publish하고 종료합니다.

```bash
ros2 run helper_cmd_vel_test cmd_vel_distance_test
```

## 회전 테스트

```bash
ros2 run helper_cmd_vel_test cmd_vel_test --ros-args \
  -p linear_x:=0.0 \
  -p angular_z:=0.3
```

## 목표 각도 회전 테스트

`cmd_vel_turn_test`는 `target_yaw_deg / angular_z`로 시간을 계산해
`/control/cmd_vel_safe`에 회전 명령을 보낸 뒤 정지하고 종료합니다. encoder나 odom을
보는 closed-loop 제어가 아니라 모터 튜닝용 open-loop 테스트입니다.

## 즉시 정지 명령

```bash
ros2 run helper_cmd_vel_test cmd_vel_stop
```

제자리 90도 좌회전:

```bash
ros2 run helper_cmd_vel_test cmd_vel_turn_test --ros-args \
  -p mode:=spin \
  -p angular_z:=0.3 \
  -p target_yaw_deg:=90.0
```

주행 중 90도 좌회전:

```bash
ros2 run helper_cmd_vel_test cmd_vel_turn_test --ros-args \
  -p mode:=arc \
  -p linear_x:=0.1 \
  -p angular_z:=0.3 \
  -p target_yaw_deg:=90.0
```

주행 중 90도 우회전:

```bash
ros2 run helper_cmd_vel_test cmd_vel_turn_test --ros-args \
  -p mode:=arc \
  -p linear_x:=0.1 \
  -p angular_z:=-0.3 \
  -p target_yaw_deg:=90.0
```

## 전진/정지/좌회전/정지/우회전/정지 반복 테스트

```bash
ros2 run helper_cmd_vel_test cmd_vel_test --ros-args \
  -p mode:=sequence \
  -p linear_x:=0.1 \
  -p angular_z:=0.3 \
  -p step_duration:=2.0
```

동일한 동작을 launch로 실행할 수도 있습니다.

```bash
ros2 launch helper_cmd_vel_test cmd_vel_constant.launch.py
ros2 launch helper_cmd_vel_test cmd_vel_sequence.launch.py
```

## Parameters

- `topic`: publish topic, default `/control/cmd_vel_safe`
- `mode`: `constant` 또는 `sequence`, default `constant`
- `publish_rate`: publish 주기 Hz, default `10.0`
- `linear_x`: 전진 속도 m/s, default `0.0`
- `angular_z`: 회전 속도 rad/s, default `0.0`
- `step_duration`: sequence 모드에서 각 단계 시간 sec, default `2.0`

`cmd_vel_distance_test` 추가 파라미터:

- `distance`: open-loop 전진 거리 m, default `0.5`
- `stop_publish_count`: 종료 전 정지 명령 publish 횟수, default `10`

`cmd_vel_stop` 추가 파라미터:

- `stop_publish_count`: 정지 명령 publish 횟수, default `20`

`cmd_vel_turn_test` 추가 파라미터:

- `mode`: `spin` 또는 `arc`, default `spin`
- `target_yaw_deg`: open-loop 목표 회전각 deg, default `90.0`
