# Vision-Language-Action (VLA) Agent for Visual Command Execution in ROS 2 Simulation

A Vision-Language-Action system that enables a mobile robot to understand 
natural language commands and execute navigation tasks in a ROS 2 Gazebo 
simulation. The system integrates a language module (Llama-3.3 70B via Groq API), 
a vision module (HSV color segmentation), and a policy module (Nav2 navigation + visual servoing) to navigate toward colored objects in a hospital environment.

![Gazeebo Environment](https://github.com/saquibattar/VLA-Agent-for-Visual-Command-Execution-in-ROS-2-Simulation/blob/main/VLA-6AttIja-ezgif.com-video-to-gif-converter.gif)

---
## Author
- Saquib Attar  — saquibattar04@gmail.com

Frankfurt University of Applied Sciences
Information Technology Course — Module AIS
Supervisor: Prof. Dr. Peter Nauth

---

## System Overview

The system has three main components:

- **Language Module** (`llm_command_node.py`): Accepts natural language commands 
  from the user, parses them using Llama-3.3 70B via Groq API, and publishes 
  action identifiers to `/robot_action` topic. Falls back to a rule-based parser 
  if the LLM is unavailable.

- **Vision Module** (`red_detector.py`): Continuously processes camera images 
  using HSV color segmentation to detect red and purple objects and publish 
  their bearing to `/red_object/bearing` and `/purple_object/bearing` topics.

- **Policy Module** (`red_object_navigator.py`): Subscribes to action and vision 
  topics, selects navigation strategy (exploration-based or action-aware), 
  coordinates with Nav2, and performs visual servoing for precise final approach.

---

## Supported Commands

| Command Example | Action |
|---|---|
| "go to the red box" | Navigate to RED object |
| "find the purple object" | Navigate to PURPLE object |
| "pass through the curtain to find the blue object" | Action-aware navigation to BLUE object |
| "stop" | Stop the robot |

---

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.10
- Gazebo (Classic)
- TurtleBot3 packages
- Nav2
- OpenCV 4.7
- Groq Python SDK

### Install dependencies

```bash
pip install groq opencv-python numpy --break-system-packages
```

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
ros-humble-turtlebot3 ros-humble-turtlebot3-gazebo \
ros-humble-cartographer ros-humble-cartographer-ros -y
```

---

## Setup

### 1. Clone the repository and build

```bash
mkdir -p ~/ws/src
cd ~/ws/src
# place your package here
cd ~/ws
colcon build
source install/setup.bash
```

### 2. Set your Groq API Key

Open `llm_command_node.py` and replace the API key:

```python
self.groq_api_key = "YOUR_GROQ_API_KEY_HERE"
```

Get a free API key from: https://console.groq.com

### 3. Set environment variables

```bash
export TURTLEBOT3_MODEL=waffle_pi
export LDS_MODEL=LDS-01
export LIBGL_ALWAYS_SOFTWARE=1
export GAZEBO_MODEL_PATH=$HOME/ws/src/vla_bringup/worlds
```

---

## Running the System

Run each command in a separate terminal. Source ROS 2 in every terminal:

```bash
source /opt/ros/humble/setup.bash
```

### Step 1 — Launch Gazebo with hospital world

```bash
export TURTLEBOT3_MODEL=waffle_pi
export LIBGL_ALWAYS_SOFTWARE=1
export GAZEBO_MODEL_PATH=$HOME/ws/src/vla_bringup/worlds
ros2 launch gazebo_ros gazebo.launch.py \
  world:=$HOME/ws/src/vla_bringup/worlds/hospital.world
```

### Step 2 — Spawn TurtleBot3

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 run gazebo_ros spawn_entity.py \
  -entity turtlebot3 \
  -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf \
  -x 0 -y 0 -z 0.15
```

### Step 3 — Launch robot drivers

```bash
export TURTLEBOT3_MODEL=waffle_pi
export LDS_MODEL=LDS-01
ros2 launch turtlebot3_bringup robot.launch.py use_sim_time:=true
```

### Step 4 — Launch Nav2 localization with map

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch nav2_bringup localization_launch.py \
  use_sim_time:=true \
  map:=$HOME/ws/src/vla_bringup/maps/hospital.yaml
```

### Step 5 — Launch Nav2 navigation

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

### Step 6 — Launch RViz2

```bash
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

### Step 7 — Run vision detector node

```bash
ros2 run vla_bringup red_detector
```

### Step 8 — Run navigation policy node

```bash
ros2 run vla_bringup red_object_navigator
```

### Step 9 — Run language command node

```bash
ros2 run vla_bringup llm_command_node
```

### Step 10 — (Optional) View camera feed

```bash
ros2 run rqt_image_view rqt_image_view
```

---

## Object Coordinates in Map Frame

| Object | x (m) | y (m) | Description |
|---|---|---|---|
| RED | 0.607 | 7.538 | Open accessible area |
| PURPLE | 7.561 | -0.768 | Open accessible area |
| BLUE | -7.076 | 7.815 | Inside closed room |
| CURTAIN | -6.0 | 5.6 | Room entrance (traversable) |

---

## HSV Color Parameters

| Color | Hmin | Hmax | Smin | Smax | Vmin | Vmax |
|---|---|---|---|---|---|---|
| RED (range 1) | 0 | 10 | 70 | 255 | 50 | 255 |
| RED (range 2) | 170 | 180 | 70 | 255 | 50 | 255 |
| PURPLE | 125 | 155 | 70 | 255 | 80 | 255 |
| BLUE | 100 | 135 | 120 | 255 | 80 | 255 |

---

## Navigation Parameters

| Parameter | Value |
|---|---|
| Exploration range | ±8.0 m |
| Adaptive step (far, d > 4m) | 1.3 m |
| Adaptive step (mid, 2-4m) | 0.9 m |
| Adaptive step (near, d < 2m) | 0.5 m |
| Visual servo trigger | 120 px height |
| Visual servo target | 260 px height |
| Center tolerance | ±40 px |
| Safety bypass radius | 3.0 m |

---

## ROS 2 Topics

| Topic | Message Type | Publisher / Subscriber |
|---|---|---|
| /camera/image_raw/compressed | sensor_msgs/CompressedImage | Camera / Vision, Policy |
| /scan | sensor_msgs/LaserScan | LiDAR / Policy |
| /robot_action | std_msgs/String | Language / Policy |
| /red_object/bearing | std_msgs/Float32 | Vision / Policy |
| /purple_object/bearing | std_msgs/Float32 | Vision / Policy |
| /cmd_vel | geometry_msgs/Twist | Policy / Robot |
| /odom | nav_msgs/Odometry | Robot / Policy |
| /tf | tf2_msgs/TFMessage | Robot / All nodes |

---

## Map Creation (First Time Only)

If you need to create the map from scratch using Cartographer SLAM:

```bash
# Launch Cartographer
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true

# Teleoperate the robot to build the map
ros2 run turtlebot3_teleop teleop_keyboard

# Save the map when done
ros2 run nav2_map_server map_saver_cli -f ~/ws/src/vla_bringup/maps/hospital
```

---

## Known Limitations

- HSV detection is sensitive to lighting conditions outside Gazebo defaults
- BLUE object is only reachable via curtain traversal command
- Object coordinates are hardcoded — relocation requires code update
- System tested only in simulation — real world transfer requires HSV retuning
- LLM requires internet connection for Groq API access

---

## License

This project was developed for academic purposes at Frankfurt University 
of Applied Sciences. All rights reserved by the author.
