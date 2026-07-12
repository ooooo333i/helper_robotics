# helper_cmd_vel_test

모터 bringup과 장애물 안전 정지를 확인하는 테스트 publisher 모음입니다.
기본 출력 `/control/cmd_vel_safe`는 실제 모터가 구독하므로 바퀴를 띄우고 비상
정지 수단을 준비한 뒤 실행하십시오.

## 빌드

```bash
cd ~/workspace/helper_robotics
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

거리와 회전 테스트는 encoder feedback이 없는 시간 기반 open-loop 테스트입니다.

확인:

```bash
ros2 topic echo /control/cmd_vel_safe
ros2 topic echo /perception/obstacle/fused
```
