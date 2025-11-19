#!/bin/bash
#
# Script to build and install the ROS 2 camera_ros package for Ubuntu 24.04
# (Jazzy) on a Raspberry Pi 5.
#
# NOTE: This script assumes you have already installed ROS 2 Jazzy following
# the official instructions.

WORKSPACE_NAME="pi_camera_ws"
CAMERA_ROS_REPO="https://github.com/christianrauch/camera_ros.git"

# Check that the correct libcamera libraries are installed
if [ ! -e /usr/local/bin/qcam ]
then
    echo "Error: libcamera does not appear to be installed."
    echo "Please run install_libcamera.bash first to build and install libcamera."
    exit 1
fi

echo "Cloning and building camera_ros in ROS workspace..."

# Install ROS 2 dependencies
sudo apt install -y \
    python3-colcon-meson \
    ros-$ROS_DISTRO-image-view

# Remove conflicting package
sudo apt remove -y \
    ros-$ROS_DISTRO-libcamera

# Put in softlinks to ROS libcamera headers for camera_ros to find them when it builds
if [ ! -e /opt/ros/$ROS_DISTRO/include/libcamera ]
then
    sudo ln -s /usr/local/include/libcamera /opt/ros/$ROS_DISTRO/include/libcamera
fi

# Create workspace
mkdir -p ~/$WORKSPACE_NAME/src
cd ~/$WORKSPACE_NAME/src

# Clone camera_ros
if [ ! -e camera_ros ]
then
    git clone $CAMERA_ROS_REPO
fi

# Build camera_ros
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/$WORKSPACE_NAME
rm -rf build install log
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera
colcon build --event-handlers=console_direct+

echo
echo "camera_ros has been built and installed in the workspace ~/$WORKSPACE_NAME."
echo "To use it, source the workspace setup file:"
echo "  source ~/$WORKSPACE_NAME/install/setup.bash"
echo "  (or add this line to your ~/.bashrc file to source it automatically)"
echo "then run the ROS 2 camera nodes like this:"
echo "  ros2 run camera_ros camera_node"
echo
echo "$0 took $SECONDS seconds."
echo
