# helper_cmd_vel_test

모터 bringup과 장애물 안전 정지를 확인하는 테스트 publisher 모음입니다.
기본 출력 토픽은 `/control/cmd_vel_safe`입니다.

## 빌드

```bash
cd ~/helper_robotics
colcon build --symlink-install --packages-select helper_msgs helper_cmd_vel_test
source install/setup.bash
```

## 실행

```bash
# 일정 속도
ros2 run helper_cmd_vel_test cmd_vel_test --ros-args \
  -p linear_x:=0.1 -p angular_z:=0.0

# 전진/회전 sequence
ros2 launch helper_cmd_vel_test cmd_vel_sequence.launch.py

# 시간 기반 0.5 m 전진
ros2 run helper_cmd_vel_test cmd_vel_distance_test

# 시간 기반 90도 제자리 회전
ros2 run helper_cmd_vel_test cmd_vel_turn_test --ros-args \
  -p mode:=spin -p angular_z:=0.3 -p target_yaw_deg:=90.0

# 즉시 정지 명령
ros2 run helper_cmd_vel_test cmd_vel_stop

# clear/obstacle 판단 반복
ros2 launch helper_cmd_vel_test obstacle_decision_sequence.launch.py
```

거리와 회전 테스트 노드는 설정 속도와 실행 시간을 이용해 시험 명령을 생성합니다.

확인:

```bash
ros2 topic echo /control/cmd_vel_safe
ros2 topic echo /perception/obstacle/fused
```
