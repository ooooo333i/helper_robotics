# helper_cmd_vel_test

`/control/cmd_vel` 모터 수신 테스트용 ROS2 패키지입니다.

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
ros2 topic echo /control/cmd_vel
```

## 전진 테스트

```bash
ros2 run helper_cmd_vel_test cmd_vel_test --ros-args \
  -p linear_x:=0.1 \
  -p angular_z:=0.0
```

## 회전 테스트

```bash
ros2 run helper_cmd_vel_test cmd_vel_test --ros-args \
  -p linear_x:=0.0 \
  -p angular_z:=0.3
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

- `topic`: publish topic, default `/control/cmd_vel`
- `mode`: `constant` 또는 `sequence`, default `constant`
- `publish_rate`: publish 주기 Hz, default `10.0`
- `linear_x`: 전진 속도 m/s, default `0.0`
- `angular_z`: 회전 속도 rad/s, default `0.0`
- `step_duration`: sequence 모드에서 각 단계 시간 sec, default `2.0`

