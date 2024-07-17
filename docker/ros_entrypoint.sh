#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/humble/setup.bash"
source "/usr/share/gazebo/setup.sh"
cd ~/ros2_ws
# colcon build
exec "$@"