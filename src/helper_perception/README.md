# helper_perception

`helper_perception`은 전·후방 LiDAR와 depth image를 가공해 Nav2용 scan/point
cloud, 장애물 판단, 주행 behavior 명령을 만드는 ROS 2 패키지입니다.

## 빌드와 실행

```bash
cd ~/workspace/helper_robotics
colcon build --symlink-install --packages-select helper_msgs helper_perception
source install/setup.bash
```

대표 실행 명령은 다음과 같습니다.

```bash
# 전방 LiDAR driver + scan filter + 거리 장애물 판단
ros2 launch helper_perception front_lidar_perception.launch.py

# 전·후방 LiDAR 두 세트
ros2 launch helper_perception dual_lidar_perception.launch.py

# depth 장애물 판단 + Nav2용 PointCloud2
ros2 launch helper_perception depth_obstacle.launch.py

# LiDAR/depth 판단 결합
ros2 launch helper_perception obstacle_fusion.launch.py

# 전체 센서 가공 묶음
ros2 launch helper_perception perception_bringup.launch.py

# Nav2 경로 위 장애물을 behavior로 변환
ros2 launch helper_perception perception_behavior_gate.launch.py
```

`perception_bringup.launch.py`는 depth camera driver 자체를 실행하지 않습니다.
먼저 별도 드라이버가 아래 topic을 발행해야 합니다.

```text
/camera/camera/depth/image_rect_raw
/camera/camera/depth/camera_info
```

LiDAR launch는 `sllidar_ros2` 패키지가 설치되어 있어야 합니다. 기본 전방 포트는
`AMR_FRONT_LIDAR_PORT` 환경 변수 또는 `/dev/ttyUSB0`, 후방은
`AMR_REAR_LIDAR_PORT` 또는 `/dev/ttyUSB2`입니다.

## 노드별 입출력과 로직

### `scan_filter_node`

| 입력 | 출력 |
|---|---|
| `/perception/scan/raw` (`sensor_msgs/msg/LaserScan`) | `/perception/scan/filtered` (`sensor_msgs/msg/LaserScan`) |

설정 각도와 거리 범위 안의 finite sample만 유지하고 나머지를 `+inf`로 바꿉니다.
메시지 header, angle, timing 정보는 보존합니다.

### `obstacle_detector_node`

| 입력 | 출력 |
|---|---|
| `/perception/scan/filtered` (`sensor_msgs/msg/LaserScan`) | `/perception/obstacle/range` (`helper_msgs/msg/ObstacleDecision`) |

검출 각도 안의 최솟값이 `obstacle_distance_threshold` 이하이면
`decision="obstacle"`, 아니면 `clear`를 발행합니다. 전방 통합 launch의 검출
영역은 장착 yaw를 반영해 `150° ~ -150°` wrap-around 구간입니다.

### `depth_obstacle_detector_node`

| 입력 | 출력 |
|---|---|
| depth image (`sensor_msgs/msg/Image`, `16UC1` 또는 `32FC1`) | `/perception/obstacle/depth` (`ObstacleDecision`) |
| camera info (`sensor_msgs/msg/CameraInfo`) | `/perception/obstacle/depth_debug` (`std_msgs/msg/String`) |

ROI의 유효 depth를 camera intrinsic, 높이, pitch로 바닥 좌표계에 투영합니다.
바닥 위 높이가 기본 0.02 m 이상인 점만 장애물 후보로 남기고, 후보가 기본
20 pixel 이상이면서 전방 거리 기본 0.8 m 이내인 상태가 3 frame 연속 확인되면
`obstacle`로 확정합니다. `clear` 전환도 기본 2 frame 확인합니다. image를
해석할 수 없거나 CameraInfo/유효 pixel이 없으면 `unknown`입니다.

`/perception/obstacle/depth_debug`에는 필터 전 `raw_decision`, 최종 `decision`,
후보 pixel 수와 frame 확인 상태가 포함됩니다. 바닥 오검출 조정은
`floor_clear_height_m`, 작은 노이즈 조정은 `min_obstacle_pixels`, 상태 전환
지연은 `obstacle_confirm_frames`와 `clear_confirm_frames`를 사용합니다.

### `depth_obstacle_cloud_node`

| 입력 | 출력 |
|---|---|
| depth image + camera info | `/perception/depth/obstacle_points` (`sensor_msgs/msg/PointCloud2`, marking) |
| depth image + camera info | `/perception/depth/clearing_points` (`sensor_msgs/msg/PointCloud2`, clearing) |

ROI pixel을 3차원으로 역투영하고 카메라 pitch/offset을 적용해 `base_link`의
`x,y,z float32` obstacle cloud로 만듭니다. 기본 0.05~0.30 m 높이만 남겨
Nav2 costmap marking에 사용합니다.

같은 유효 ROI depth를 카메라 optical 좌표 `(x=오른쪽, y=아래, z=전방)`로도
만들어 clearing cloud로 발행합니다. 실제 Nav2 local costmap은 obstacle cloud를
`depth_mark`, clearing cloud를 `depth_clear` source로 사용합니다. global
costmap에는 전방 LiDAR scan만 들어갑니다.

현재 cloud 구현은 ROI, 거리, 높이, sampling 필터까지만 적용합니다. 작은 cluster
제거, 여러 frame 연속 확인, confidence 계산은 아직 없으므로 실제 주행 전 camera
높이/pitch를 실측하고 빈 바닥에서 오검출 여부를 확인해야 합니다.

카메라 장착값은 정면 장착 기준으로 detector와 cloud 모두 `x=0.1847 m`,
`height=0.1889 m`, `pitch=0°`를 사용합니다. `camera_pitch_deg`는 수평 기준
optical axis 하향각이므로 카메라를 다시 기울이면 URDF와 두 depth 노드의 값을
같이 수정해야 합니다.

### `obstacle_fusion_node`

| 입력 | 출력 |
|---|---|
| `/perception/obstacle/range` (`ObstacleDecision`) | `/perception/obstacle/fused` (`ObstacleDecision`) |
| `/perception/obstacle/depth` (`ObstacleDecision`) | |

0.5초 이내의 fresh 입력만 사용하며 우선순위는 `obstacle > unknown > clear`입니다.
`config/fusion.yaml`에서는 두 입력이 모두 stale이면 `unknown`을 발행하고,
최종 decision에 기여한 입력 중 거리값은 가능하면 LiDAR 값을 우선합니다.
높이는 기여 입력의 최댓값, `is_dynamic`은 하나라도 true이면 true로 보존합니다.

### `obstacle_action_node`

| 입력 | 출력 |
|---|---|
| range/depth `ObstacleDecision` | `/perception/obstacle/action` (`std_msgs/msg/String`) |

각 센서별 stop/turn 거리와 depth 높이 범위를 적용해 `stop`, `turn`, `go` 중
하나를 10 Hz로 발행합니다. 두 센서가 모두 stale이면 기본 `stop`입니다.

### `perception_behavior_gate_node`

| 구분 | topic | 타입 |
|---|---|---|
| 입력 | `/local_plan` | `nav_msgs/msg/Path` |
| 입력 | `/perception/scan/filtered` | `sensor_msgs/msg/LaserScan` |
| 입력 | `/perception/obstacle/depth` | `helper_msgs/msg/ObstacleDecision` |
| 입력 | `/control/odom` | `nav_msgs/msg/Odometry` |
| 출력 | `/planning/behavior_cmd` | `std_msgs/msg/String` |
| 출력 | `/perception/obstacle/dynamic` | `std_msgs/msg/Bool` |
| 출력 | `/perception/obstacle/dynamic_speed` | `std_msgs/msg/Float32` |

fresh한 local path를 `base_link`로 변환하고, LiDAR point를 cluster로 묶은 뒤
경로 2 m 앞/폭 0.5 m 안의 장애물을 찾습니다. 연속 거리 변화와 로봇 속도로
closing speed/TTC를 계산합니다.

- 동적 장애물 또는 TTC 위험: `stop`
- 경로 위 정적 LiDAR 장애물: costmap에 맡기고 `run`
- depth 높이 0.02 m 미만 또는 유효하지 않음: `run`
- depth 높이 0.02 m 이상 0.05 m 미만: 동적/TTC 위험이 아니면 `overcome`;
  costmap에서는 제외
- depth 높이 0.05 m 이상: costmap에 맡기고 `run`
- 장애물이 없으면: `run`

stop chatter를 줄이기 위해 기본 0.8초 최소 유지와 2초 clear 확인 latch를
사용합니다. `overcome`은 장애물이 사라진 뒤에도 기본 3초 유지됩니다. path가
stale이면 behavior 명령을 발행하지 않습니다.

`/perception/obstacle/fused`는 LiDAR/depth의 단순 상태 결합 결과입니다. 현재
Nav2 costmap과 `perception_behavior_gate_node`는 이 fused topic을 사용하지
않습니다. 기본 motor launch도 obstacle safety를 비활성화하므로 fused 결과가
기본 주행을 직접 정지시키지는 않습니다.

## Launch 구성

| launch | 포함 구성 |
|---|---|
| `front_lidar_c1.launch.py` | 전방 SLLIDAR → `/perception/scan/raw` |
| `front_lidar_slam.launch.py` | 전방 SLLIDAR + SLAM용 filter |
| `front_lidar_perception.launch.py` | 전방 driver + filter + detector |
| `rear_lidar_perception.launch.py` | 후방 driver + filter + detector |
| `dual_lidar_perception.launch.py` | 전방 + 후방 perception |
| `depth_obstacle.launch.py` | depth decision + point cloud |
| `depth_to_scan.launch.py` | depth image → `/perception/scan/depth` |
| `perception_bringup.launch.py` | 전방 LiDAR + depth + fusion + depth scan |

## 주요 설정

- `config/lidar.yaml`: scan 각도/거리와 LiDAR 장애물 임계값
- `config/depth.yaml`: depth ROI, 거리, 카메라 장착값, cloud 높이
- `config/fusion.yaml`: 입력 timeout과 fusion 우선순위
- `config/action.yaml`: `stop/turn/go` 거리
- `config/behavior_gate.yaml`: 경로 폭, TTC, 동적 판단, stop latch

## Nav2 costmap 연결

실제 로봇 설정 `helper_navigation/config/helper_nav2_params.yaml`의 local
`VoxelLayer`는 다음 세 입력을 사용합니다.

```text
observation_sources: scan depth_mark depth_clear

scan:
  /perception/scan/filtered
  sensor_msgs/msg/LaserScan

depth_mark:
  /perception/depth/obstacle_points
  sensor_msgs/msg/PointCloud2
  높이 0.05~0.30 m
  marking=true, clearing=false

depth_clear:
  /perception/depth/clearing_points
  sensor_msgs/msg/PointCloud2
  frame=CameraInfo.header.frame_id (일반적으로 camera_depth_optical_frame)
  marking=false, clearing=true
  raytrace=0.2~2.0 m
```

`base_link -> depth_camera_link -> camera_link`는 URDF가 제공하고,
`camera_link -> camera_depth_optical_frame`은 depth camera driver TF를
사용합니다. 이 TF가 없으면 clearing source가 costmap에 반영되지 않습니다.
처음에는 RViz에서 두 cloud 좌표와 장애물 제거 후 잔류 여부를 확인한 뒤
실주행해야 합니다.

## 확인 명령

```bash
ros2 topic echo /perception/scan/filtered --once
ros2 topic echo /perception/obstacle/range
ros2 topic echo /perception/obstacle/depth
ros2 topic echo /perception/depth/obstacle_points --once
ros2 topic echo /perception/depth/clearing_points --once
ros2 topic echo /perception/obstacle/fused
ros2 topic echo /planning/behavior_cmd
```
