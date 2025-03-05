#!/bin/bash

# Source the ROS 2 Humble setup file
source /opt/ros/humble/setup.bash

colcon build

# Source the ROS 2 workspace setup file
source install/setup.bash

# Run the ROS 2 nodes
ros2 run rgb blob_detect move_to_color