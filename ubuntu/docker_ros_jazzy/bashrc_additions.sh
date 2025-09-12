#!/bin/bash
# ROS Jazzy environment setup for .bashrc
# Add this to your .bashrc or source this file directly

# Source ROS Jazzy environment
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "ROS Jazzy environment sourced"
fi

# Source workspace if it exists
if [ -f ~/ros_ws/install/setup.bash ]; then
    source ~/ros_ws/install/setup.bash
    echo "ROS workspace environment sourced"
fi

# Set ROS environment variables
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}

# Useful ROS aliases
alias ros2_build='colcon build --symlink-install'
alias ros2_source='source install/setup.bash'
alias ros2_clean='rm -rf build/ install/ log/'
alias ros2_test='colcon test && colcon test-result --verbose'

# Add ROS tools to PATH if they exist
if [ -d "/opt/ros/jazzy/bin" ]; then
    export PATH="/opt/ros/jazzy/bin:$PATH"
fi

echo "ROS Jazzy environment configured!"
echo "Available aliases: ros2_build, ros2_source, ros2_clean, ros2_test"