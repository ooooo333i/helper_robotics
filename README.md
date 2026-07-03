# Helper Robotics

Helper Robotics is a ROS 2 workspace for an indoor mobile robot. It includes
LiDAR and depth-camera perception, SLAM and Nav2 navigation, behavior control,
an MD200T motor driver, and a minimal VDA5050 MQTT adapter.

**Target environment:** Ubuntu 22.04, ROS 2 Humble, Python 3.10

## Packages

| Package | Purpose |
|---|---|
| `helper_msgs` | Custom ROS 2 messages |
| `helper_description` | Robot URDF and sensor transforms |
| `helper_perception` | LiDAR/depth processing and obstacle decisions |
| `helper_navigation` | SLAM, localization, Nav2, and behavior management |
| `helper_control` | Velocity safety gate and MD200T motor control |
| `helper_status` | Robot-status publisher |
| `helper_vda5050` | VDA5050-to-ROS 2 MQTT adapter |
| `helper_cmd_vel_test` | Motor and safety-gate test publishers |

## Installation

Install ROS 2 Humble Desktop first, then install the workspace tools:

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool

sudo rosdep init
rosdep update
```

`sudo rosdep init` is only required once per machine.

Clone the workspace and import the external LiDAR driver:

```bash
mkdir -p ~/workspace
cd ~/workspace
git clone https://github.com/ooooo333i/helper_robotics.git
cd helper_robotics
vcs import src < dependencies.repos
```

Install dependencies and build:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install
source install/setup.bash
```

Source ROS 2 and the workspace in every new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/helper_robotics/install/setup.bash
```

## Run Modes

### 1. Software-Only Navigation Demo

This demo uses fake scan and odometry data. It does not require sensors,
motors, or a physics simulator.

```bash
ros2 launch helper_navigation behavior_nav2_demo.launch.py rviz:=true
```

In RViz, use **2D Goal Pose** to send a navigation goal.

Behavior commands can be tested from another terminal:

```bash
ros2 topic pub --times 3 /planning/behavior_cmd \
  std_msgs/msg/String "{data: stop}"

ros2 topic pub --times 3 /planning/behavior_cmd \
  std_msgs/msg/String "{data: run}"
```

### 2. Real-Robot Operation

> Test with the drive wheels lifted first and keep an emergency-stop method
> available.

Configure the connected USB devices:

```bash
cd ~/workspace/helper_robotics
./scripts/usb_port_setup.sh scan
./scripts/usb_port_setup.sh configure
source config/usb_ports.env
./scripts/usb_port_setup.sh check
```

If serial access is denied, add the current user to `dialout` and log in again:

```bash
sudo usermod -aG dialout "$USER"
```

#### SLAM and Map Saving

Validate LiDAR and SLAM without motor output:

```bash
ros2 launch helper_navigation slam_bringup.launch.py \
  motor:=false \
  rviz:=true
```

Enable the motor after validation:

```bash
ros2 launch helper_navigation slam_bringup.launch.py \
  motor:=true \
  rviz:=true
```

Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f \
  ~/workspace/helper_robotics/src/helper_navigation/maps/helper_map
```

#### Navigation on a Saved Map

```bash
source ~/workspace/helper_robotics/config/usb_ports.env

ros2 launch helper_navigation map_navigation.launch.py \
  map:=$HOME/workspace/helper_robotics/src/helper_navigation/maps/helper_map.yaml \
  front_lidar_port:=${AMR_FRONT_LIDAR_PORT} \
  motor_port:=${AMR_MOTOR_DRIVER_PORT} \
  motor:=true \
  rviz:=true
```

In RViz, set the initial position with **2D Pose Estimate**, then send a goal
with **2D Goal Pose**.

Run depth-camera perception and automatic behavior decisions in two additional
terminals:

```bash
ros2 launch helper_perception depth_obstacle.launch.py
```

```bash
ros2 launch helper_perception perception_behavior_gate.launch.py
```

### 3. Virtual VDA5050 Demo

Install and start the local MQTT broker:

```bash
sudo apt install -y mosquitto mosquitto-clients
sudo systemctl enable --now mosquitto
```

Terminal 1 — software-only navigation:

```bash
ros2 launch helper_navigation behavior_nav2_demo.launch.py rviz:=true
```

Terminal 2 — VDA5050 adapter:

```bash
ros2 launch helper_vda5050 vda5050_adapter.launch.py \
  broker_host:=localhost
```

Terminal 3 — browser demo panel:

```bash
ros2 launch helper_vda5050 vda5050_demo_panel.launch.py \
  broker_host:=localhost
```

Open <http://127.0.0.1:8088> to send navigation orders and instant actions.

