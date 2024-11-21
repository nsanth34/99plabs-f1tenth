#!/bin/bash
set -e

# Load env variables
set -a
source "/project_ws/install/envfile"
set +a

# Setup ROS2 environment
source "/project_ws/install/venv/bin/activate"
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/project_ws/install/local_setup.bash"
ros2 launch f1tenth_stack bringup_launch.py