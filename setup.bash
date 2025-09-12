#!/bin/bash
# Comprehensive Raspberry Pi setup script for robotics development
# This script sets up a complete development environment

# Stop on first error
set -e

echo
echo "====================================="
echo "Raspberry Pi Robotics Setup Script"
echo "====================================="
echo "This script will set up your Raspberry Pi for robotics development."
echo "The setup includes Docker, ROS, camera support, and development tools."
echo

# Update system first
echo "Updating system packages..."
sudo apt update && sudo apt upgrade -y

# Part 1: Installation and configuration requiring logout/login
echo
echo "Phase 1: Installing Docker and configuring user groups..."
echo

# Install Docker
if [ -f "./install_docker.bash" ]; then
    ./install_docker.bash
else
    echo "Warning: install_docker.bash not found"
fi

# Add user to video group (for camera access)
echo "Adding user $USER to video group for camera access..."
sudo usermod -aG video $USER

echo
echo "========================================"
echo "IMPORTANT: LOGOUT AND LOGIN REQUIRED"
echo "========================================"
echo "You must log out and log back in for the group changes to take effect."
echo "After logging back in, this setup will continue with the remaining tasks."
echo
echo "To continue setup after logging back in, run:"
echo "  ./setup.bash --continue"
echo
echo "Press any key to exit so you can logout and login..."
read -n 1 -s

# Check if we're in continuation mode
if [[ "$1" == "--continue" ]]; then
    echo
    echo "Continuing with Phase 2 setup..."
else
    echo "Exiting so you can logout and login. Run './setup.bash --continue' afterwards."
    exit 0
fi

# Part 2: Remaining setup tasks (after logout/login)
echo
echo "Phase 2: Building containers and setting up development environment..."
echo

# Verify Docker access
if ! docker info &>/dev/null; then
    echo "Error: Cannot access Docker. Please make sure you logged out and back in."
    exit 1
fi

# Build Docker container
echo "Building development container..."
if [ -f "./build_container.bash" ]; then
    ./build_container.bash
else
    echo "Warning: build_container.bash not found"
fi

# Setup X11 forwarding
echo "Setting up X11 forwarding for GUI applications..."
if [ -f "./setup_x11.bash" ]; then
    ./setup_x11.bash
else
    echo "Warning: setup_x11.bash not found"
fi

# Create development directories
echo "Creating development directories..."
mkdir -p ~/git
mkdir -p ~/robotics_workspace/src
mkdir -p ~/data/logs
mkdir -p ~/data/images

# Test camera if available
echo "Testing camera functionality..."
if [ -f "./test_camera.bash" ]; then
    ./test_camera.bash
else
    echo "Warning: test_camera.bash not found. You can install camera support later with:"
    echo "  ./ubuntu/pi_camera/install_libcamera.bash"
fi

# Create example files
echo "Creating example configuration files..."

# Create a sample ROS launch file
mkdir -p ~/robotics_workspace/src/examples/launch
cat << 'EOF' > ~/robotics_workspace/src/examples/launch/example.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker'
        ),
        Node(
            package='demo_nodes_py',
            executable='listener',
            name='listener'
        )
    ])
EOF

# Create a sample Docker compose file for multi-container setups
cat << 'EOF' > docker-compose.yml
version: '3.8'

services:
  robotics:
    build: .
    image: rpi-robotics:latest
    container_name: rpi-robotics-dev
    volumes:
      - ./robotics_workspace:/ros_ws
      - /dev:/dev
    privileged: true
    network_mode: host
    environment:
      - DISPLAY=${DISPLAY}
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
EOF

# Create basic .bashrc additions
echo "Setting up shell environment..."
cat << 'EOF' >> ~/.bashrc

# Robotics development aliases
alias drun='docker run -it --rm --privileged --net=host -v $(pwd):/workspace -w /workspace'
alias dcup='docker-compose up -d'
alias dcdown='docker-compose down'
alias rosws='cd ~/robotics_workspace'

# ROS environment (if running outside container)
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi
if [ -f "~/robotics_workspace/install/setup.bash" ]; then
    source ~/robotics_workspace/install/setup.bash
fi
EOF

echo
echo "================================================="
echo "Setup completed successfully!"
echo "================================================="
echo
echo "Summary of what was installed:"
echo "- Docker and Docker Compose"
echo "- X11 forwarding for GUI applications"
echo "- Development directories structure"
echo "- ROS development container"
echo "- Camera support (if available)"
echo "- Example configuration files"
echo
echo "Next steps:"
echo "1. Source your updated .bashrc: source ~/.bashrc"
echo "2. Test the setup with: docker run hello-world"
echo "3. Run GUI container with: ./run_gui_container.bash"
echo "4. Start developing in ~/robotics_workspace/"
echo
echo "Happy coding!"
echo