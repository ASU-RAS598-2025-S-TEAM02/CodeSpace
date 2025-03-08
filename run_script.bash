#!/bin/bash

# Source the ROS 2 Humble setup file
source /opt/ros/humble/setup.bash

cd ~/ros2_ws

colcon build

# Source the ROS 2 workspace setup file
source install/setup.bash

# Open TMUX session
tmux new-session -d -s my_session "ros2 launch depthai_ros_driver camera.launch.py params_file:="/home/$USER/camera.yaml" namespace:='/rpi_05'"

# Run the ROS 2 nodes
ros2 run rgb blob_detect move