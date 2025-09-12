#!/bin/bash
# Install ROS Jazzy on Ubuntu 24.04 LTS for Raspberry Pi
# This script installs ROS Jazzy natively (not in Docker)

# Stop on first error
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Installing ROS Jazzy on Ubuntu 24.04 LTS...${NC}"

# Check Ubuntu version
if ! lsb_release -r | grep -q "24.04"; then
    echo -e "${YELLOW}Warning: This script is designed for Ubuntu 24.04 LTS${NC}"
    echo -e "${YELLOW}ROS Jazzy officially supports Ubuntu 24.04 LTS${NC}"
fi

# Update package index
echo -e "${GREEN}Updating package index...${NC}"
sudo apt update

# Install required tools
echo -e "${GREEN}Installing required tools...${NC}"
sudo apt install -y \
    curl \
    gnupg \
    lsb-release \
    software-properties-common

# Add ROS 2 GPG key
echo -e "${GREEN}Adding ROS 2 GPG key...${NC}"
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository
echo -e "${GREEN}Adding ROS 2 repository...${NC}"
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package index with new repository
echo -e "${GREEN}Updating package index with ROS repository...${NC}"
sudo apt update

# Install ROS Jazzy Desktop
echo -e "${GREEN}Installing ROS Jazzy Desktop...${NC}"
sudo apt install -y ros-jazzy-desktop

# Install additional development tools
echo -e "${GREEN}Installing additional development tools...${NC}"
sudo apt install -y \
    python3-pip \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    python3-colcon-common-extensions

# Initialize rosdep
echo -e "${GREEN}Initializing rosdep...${NC}"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# Create workspace directory
echo -e "${GREEN}Creating ROS workspace...${NC}"
mkdir -p ~/ros_ws/src

# Add ROS environment to bashrc
echo -e "${GREEN}Setting up environment...${NC}"
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS Jazzy environment" >> ~/.bashrc
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi

# Add workspace setup to bashrc if workspace exists
if ! grep -q "source ~/ros_ws/install/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# Source ROS workspace if it exists" >> ~/.bashrc
    echo "if [ -f ~/ros_ws/install/setup.bash ]; then" >> ~/.bashrc
    echo "    source ~/ros_ws/install/setup.bash" >> ~/.bashrc
    echo "fi" >> ~/.bashrc
fi

# Add useful aliases
if ! grep -q "alias ros2_build" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS 2 aliases" >> ~/.bashrc
    echo "alias ros2_build='colcon build --symlink-install'" >> ~/.bashrc
    echo "alias ros2_source='source install/setup.bash'" >> ~/.bashrc
    echo "alias ros2_clean='rm -rf build/ install/ log/'" >> ~/.bashrc
    echo "alias ros2_test='colcon test && colcon test-result --verbose'" >> ~/.bashrc
fi

# Source the environment for current session
source /opt/ros/jazzy/setup.bash

echo -e "${GREEN}ROS Jazzy installation complete!${NC}"
echo ""
echo "To use ROS Jazzy in your current terminal, run:"
echo "  source /opt/ros/jazzy/setup.bash"
echo ""
echo "Or open a new terminal (environment is now in .bashrc)"
echo ""
echo "Test the installation with:"
echo "  ros2 run demo_nodes_cpp talker"
echo ""
echo "Create your first package:"
echo "  cd ~/ros_ws/src"
echo "  ros2 pkg create --build-type ament_python my_package"
echo "  cd ~/ros_ws"
echo "  colcon build"
echo "  source install/setup.bash"