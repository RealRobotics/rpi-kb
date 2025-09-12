#!/bin/bash
set -e

# Source ROS Jazzy environment
source /opt/ros/jazzy/setup.bash

# Source workspace if it exists
if [ -f /opt/ros_ws/devel/setup.bash ]; then
    source /opt/ros_ws/devel/setup.bash
fi

# Execute the command passed to the script
exec "$@"