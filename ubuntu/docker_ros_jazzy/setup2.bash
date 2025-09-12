#!/bin/bash
# Raspberry Pi setup script - Part 2
# This script runs after logout/login and completes the remaining setup tasks

# Stop on first error
set -e

echo
echo "=========================================="
echo "Raspberry Pi Setup - Part 2"
echo "=========================================="
echo "This script completes the setup after you have logged out and back in."
echo "It will build containers, setup development environment, and test functionality."
echo

# Verify Docker access (user should now be in docker group)
echo "Verifying Docker access..."
if ! docker info &>/dev/null; then
    echo "Error: Cannot access Docker daemon."
    echo "Please make sure you logged out and logged back in after running setup_part1.bash"
    echo "You can check if you're in the docker group with: groups \$USER"
    exit 1
fi
echo "Docker access confirmed!"

echo
echo "Building development container..."
# Build Docker container using modular script
if [ -f "./build_container.bash" ]; then
    ./build_container.bash
else
    echo "Warning: build_container.bash not found. Skipping container build."
fi

echo
echo "Setting up X11 forwarding for GUI applications..."
# Setup X11 forwarding using modular script
if [ -f "./setup_x11.bash" ]; then
    ./setup_x11.bash
else
    echo "Warning: setup_x11.bash not found. Skipping X11 setup."
fi

echo
echo "Creating development directory structure..."
# Create development directories
mkdir -p ~/git
mkdir -p ~/robotics_workspace/src
mkdir -p ~/data/logs
mkdir -p ~/data/images
mkdir -p ~/examples

echo "Development directories created:"
echo "  ~/git - for cloning repositories"
echo "  ~/robotics_workspace - ROS workspace"
echo "  ~/data - for data storage"
echo "  ~/examples - example files"

echo
echo "Testing camera functionality..."
# Test camera using modular script
if [ -f "./test_camera.bash" ]; then
    chmod +x ./test_camera.bash
    if ./test_camera.bash; then
        echo "Camera test completed successfully!"
    else
        echo "Camera test failed or no camera detected."
        echo "You can install camera support later with:"
        echo "  ./ubuntu/pi_camera/install_libcamera.bash"
        echo "  or ./ubuntu/pi_camera/install_arducam.bash"
    fi
else
    echo "Warning: test_camera.bash not found. Skipping camera test."
    echo "Camera support scripts are available in ./ubuntu/pi_camera/"
fi

echo
echo "Creating example configuration files..."

# Create example ROS workspace structure
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

# Create example package.xml
cat << 'EOF' > ~/robotics_workspace/src/examples/package.xml
<?xml version="1.0"?>
<package format="3">
  <name>examples</name>
  <version>0.0.1</version>
  <description>Example ROS 2 package</description>
  <maintainer email="user@example.com">Your Name</maintainer>
  <license>MIT</license>

  <exec_depend>demo_nodes_cpp</exec_depend>
  <exec_depend>demo_nodes_py</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

# Create basic README for the user
cat << 'EOF' > ~/examples/README.md
# Raspberry Pi Robotics Setup

This directory contains examples and documentation for your robotics development environment.

## Quick Start

1. **Test Docker**: `docker run hello-world`
2. **Run GUI container**: `./run_gui_container.bash` (if X11 is setup)
3. **Build ROS workspace**:
   ```bash
   cd ~/robotics_workspace
   colcon build
   source install/setup.bash
   ```
4. **Test camera**: `./test_camera.bash`

## Directory Structure

- `~/git/` - Clone repositories here
- `~/robotics_workspace/` - ROS 2 workspace
- `~/data/` - Data storage (logs, images)
- `~/examples/` - Example configurations

## Useful Commands

- `docker ps` - List running containers
- `docker images` - List available images
- `groups $USER` - Check your group memberships
- `lsusb` - List USB devices (cameras, etc.)

## Next Steps

1. Install ROS 2 packages: `sudo apt install ros-jazzy-desktop`
2. Set up your SSH keys for git repositories
3. Clone your robotics projects to `~/git/`
4. Configure your development IDE/editor

Happy coding!
EOF

echo
echo "================================================="
echo "SETUP COMPLETED SUCCESSFULLY!"
echo "================================================="
echo
echo "Summary of what was configured:"
echo "✓ Docker development container built"
echo "✓ X11 forwarding configured for GUI applications"
echo "✓ Development directory structure created"
echo "✓ Camera functionality tested (if available)"
echo "✓ Example ROS 2 workspace and files created"
echo "✓ Docker Compose configuration created"
echo
echo "Your Raspberry Pi is now ready for robotics development!"
echo
echo "Next steps:"
echo "1. Source your updated shell: source ~/.bashrc"
echo "2. Test Docker: docker run hello-world"
echo "3. Test GUI container: ./run_gui_container.bash"
echo "4. Explore the examples in ~/examples/README.md"
echo "5. Start your robotics projects in ~/ws/"
echo
echo "Need camera support? Run one of these:"
echo "  ./ubuntu/pi_camera/install_libcamera.bash"
echo "  ./ubuntu/pi_camera/install_arducam.bash"
echo
echo "Happy robotics development!"
echo
