#!/bin/bash
# Raspberry Pi setup script - Part 2
# This script runs after logout/login and completes the remaining setup tasks

# Stop on first error
set -e

docker_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &>/dev/null && pwd )"
. ${docker_dir}/vars.bash

echo
echo "Creating development directory structure..."
# Create development directories
mkdir -p ~/git
mkdir -p ${WORKSPACE_DIR}/src
mkdir -p ~/data/logs
mkdir -p ~/data/images
mkdir -p ~/examples

echo "Development directories created:"
echo "  ~/git - for cloning repositories"
echo "  ${WORKSPACE_DIR} - ROS workspace"
echo "  ~/data - for data storage"
echo "  ~/examples - example files"

echo
echo "Creating example configuration files..."

# Create example ROS workspace structure
mkdir -p ${WORKSPACE_DIR}/src/examples/launch
cat << 'EOF' > ${WORKSPACE_DIR}/src/examples/launch/example.launch.py
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
cat << 'EOF' > ${WORKSPACE_DIR}/src/examples/package.xml
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
   cd ${WORKSPACE_DIR}
   colcon build
   source install/setup.bash
   ```
4. **Test camera**: `./test_camera.bash`

## Directory Structure

- `~/git/` - Clone repositories here
- `${WORKSPACE_DIR}/` - ROS 2 workspace
- `~/data/` - Data storage (logs, images)
- `~/examples/` - Example configurations

## Useful Commands

- `docker ps` - List running containers
- `docker images` - List available images
- `groups $USER` - Check your group memberships
- `lsusb` - List USB devices (cameras, etc.)

## Next Steps

1. Install ROS 2 packages as needed.
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
echo "Next steps:"
echo "1. Source your updated shell: source ~/.bashrc"
echo "2. Test Docker: docker run hello-world"
echo "3. Test GUI container: ./start.bash then ./attach.bash"
echo "4. Explore the examples in ~/examples/README.md"
echo "5. Start your robotics projects in ${WORKSPACE_DIR}/"
echo
echo
