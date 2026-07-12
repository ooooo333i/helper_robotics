# helper_perception

전·후방 LiDAR와 depth camera 데이터를 가공해 Nav2용 scan/point cloud,
장애물 판단, 주행 behavior 명령을 생성합니다.

## 빌드

```bash
cd ~/workspace/helper_robotics
colcon build --symlink-install --packages-select helper_msgs helper_perception
source install/setup.bash
```

## 실행

```bash
# 전방 LiDAR + 필터 + 장애물 판단
ros2 launch helper_perception front_lidar_perception.launch.py

# 전·후방 LiDAR
ros2 launch helper_perception dual_lidar_perception.launch.py

# RealSense + depth 장애물 판단 + Nav2 point cloud
ros2 launch helper_perception depth_obstacle.launch.py

# LiDAR/depth 판단 결합
ros2 launch helper_perception obstacle_fusion.launch.py

# 전체 perception 구성
ros2 launch helper_perception perception_bringup.launch.py

# 경로 위 장애물을 behavior 명령으로 변환
ros2 launch helper_perception perception_behavior_gate.launch.py
```

LiDAR launch에는 `sllidar_ros2`, depth launch에는 RealSense ROS 2 driver가
필요합니다. 포트는 다음 환경 변수로 지정할 수 있습니다.

```bash
export AMR_FRONT_LIDAR_PORT=/dev/serial/by-id/<front-lidar>
export AMR_REAR_LIDAR_PORT=/dev/serial/by-id/<rear-lidar>
```

## 주요 흐름

```text
LiDAR -> /perception/scan/filtered -> range obstacle
Depth -> obstacle/clearing PointCloud2 + depth obstacle
range + depth -> /perception/obstacle/fused
path + scan + depth + odom -> /planning/behavior_cmd
```

설정 파일은 `config/lidar.yaml`, `depth.yaml`, `fusion.yaml`,
`behavior_gate.yaml`입니다.

```bash
ros2 topic echo /perception/scan/filtered --once
ros2 topic echo /perception/obstacle/depth
ros2 topic echo /perception/obstacle/fused
ros2 topic echo /planning/behavior_cmd
```
