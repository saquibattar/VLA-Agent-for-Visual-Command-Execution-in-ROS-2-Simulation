Step 1 -
killall -9 gzserver gzclient rviz2

Step 2 -
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
export LIBGL_ALWAYS_SOFTWARE=1

ros2 launch gazebo_ros gazebo.launch.py \
  world:=$HOME/ws/src/vla_bringup/worlds/hospital.world


Or -
source /opt/ros/humble/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export GAZEBO_MODEL_PATH=$HOME/ws/src/vla_bringup/worlds
export TURTLEBOT3_MODEL=waffle_pi

ros2 launch gazebo_ros gazebo.launch.py \
  world:=$HOME/ws/src/vla_bringup/worlds/hospital.world

Step 3 -
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

ros2 run gazebo_ros spawn_entity.py \
  -entity turtlebot3 \
  -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf \
  -x 0 -y 0 -z 0.15

Step 4 -
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
export LDS_MODEL=LDS-01

ros2 launch turtlebot3_bringup robot.launch.py use_sim_time:=true


Step 5 -
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

ros2 launch nav2_bringup localization_launch.py \
  use_sim_time:=true \
  map:=$HOME/ws/src/vla_bringup/maps/hospital.yaml


Step 6 -
ros2 topic echo /map --once


Step 7 -
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true


Step 8 -
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz


Step 9 -
ros2 run vla_bringup red_detector


Step 10 -
ros2 run vla_bringup red_object_navigator


Step 11 -
ros2 run vla_bringup llm_command_node

cd ~/ws
colcon build
source install/setup.bash

source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 run rqt_image_view rqt_image_view