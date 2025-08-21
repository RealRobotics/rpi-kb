#!/bin/bash
# Complete setup script for Docker with ROS 2 Jazzy on Raspberry Pi 5

# Stop on first error.
set -e

# Tell the user what is going on.
echo
echo "=== Raspberry Pi 5 Docker + ROS 2 Jazzy Setup ==="
echo "This script will install Docker and build a container with:"
echo "  - Ubuntu 24.04LTS"
echo "  - ROS 2 Jazzy"
echo "  - Camera support"
echo "  - Development tools"
echo

# Check if running on Raspberry Pi
if ! grep -q "Raspberry Pi" /proc/device-tree/model 2>/dev/null; then
    echo "Warning: This script is designed for Raspberry Pi. Continue? (y/N)"
    read -r response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        echo "Aborting setup."
        exit 1
    fi
fi

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "Script directory: $SCRIPT_DIR"
echo

# Step 1: Install Docker
echo "=== Step 1: Installing Docker ==="
if command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1; then
    echo "Docker is already installed and running."
else
    echo "Installing Docker..."
    "$SCRIPT_DIR/install_docker.bash"
    
    echo
    echo "Docker installation complete. You may need to log out and back in"
    echo "for group membership changes to take effect."
    echo
    echo "Testing Docker installation..."
    if sudo docker run --rm hello-world >/dev/null 2>&1; then
        echo "✓ Docker is working correctly"
    else
        echo "✗ Docker test failed. Please check the installation."
        exit 1
    fi
fi

# Step 2: Build the ROS 2 container
echo
echo "=== Step 2: Building ROS 2 Jazzy container ==="
echo "This may take 30-60 minutes depending on your internet connection..."
echo

"$SCRIPT_DIR/build_container.bash"

# Step 3: Create host directories
echo
echo "=== Step 3: Setting up host directories ==="
HOST_WORKSPACE="$HOME/ros2_ws"
HOST_DATA="$HOME/docker_data"

mkdir -p "$HOST_WORKSPACE/src"
mkdir -p "$HOST_DATA"

echo "Created directories:"
echo "  - Workspace: $HOST_WORKSPACE"
echo "  - Data: $HOST_DATA"

# Step 4: Set up X11 forwarding
echo
echo "=== Step 4: Setting up X11 forwarding ==="
XAUTH_FILE=/tmp/.docker.xauth

if [ ! -f "$XAUTH_FILE" ]; then
    touch "$XAUTH_FILE"
    chmod 666 "$XAUTH_FILE"
fi

if [ -n "$DISPLAY" ]; then
    xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH_FILE" nmerge - 2>/dev/null || true
    echo "X11 forwarding configured for display: $DISPLAY"
else
    echo "Warning: DISPLAY not set. GUI applications may not work."
fi

# Step 5: Test camera access
echo
echo "=== Step 5: Testing camera access ==="
echo "Camera devices found:"
ls -la /dev/video* 2>/dev/null || echo "No camera devices detected"

echo
echo "User groups:"
groups "$USER"

if groups "$USER" | grep -q "video"; then
    echo "✓ User is in video group"
else
    echo "⚠ User is not in video group. Camera access may be limited."
fi

# Step 6: Create example files
echo
echo "=== Step 6: Creating example files ==="

# Copy sample camera publisher to workspace
cp "$SCRIPT_DIR/camera_publisher.py" "$HOST_WORKSPACE/src/" 2>/dev/null || echo "Note: camera_publisher.py not copied"

# Create a simple README in the workspace
cat > "$HOST_WORKSPACE/README.md" << 'EOF'
# ROS 2 Workspace

This workspace is shared between your host system and the Docker container.

## Quick Start

1. Start the container:
   ```bash
   ./run_container.bash
   ```

2. Inside the container, test the camera:
   ```bash
   ./test_camera.bash
   ```

3. Run a camera node:
   ```bash
   ros2 run image_tools cam2image
   ```

4. View camera topics:
   ```bash
   ros2 topic list
   ros2 topic echo /image
   ```

## Files

- `camera_publisher.py` - Example camera publisher node
- `src/` - Place your ROS 2 packages here
EOF

echo "Created workspace README: $HOST_WORKSPACE/README.md"

# Final summary
echo
echo "=== Setup Complete! ==="
echo
echo "Summary:"
echo "✓ Docker installed and configured"
echo "✓ ROS 2 Jazzy container built"
echo "✓ Host directories created"
echo "✓ X11 forwarding configured"
echo "✓ Example files created"
echo
echo "Next steps:"
echo "1. Start the container:"
echo "   cd $SCRIPT_DIR"
echo "   ./run_container.bash"
echo
echo "2. Inside the container, test the camera:"
echo "   ./test_camera.bash"
echo
echo "3. Try running a camera node:"
echo "   ros2 run image_tools cam2image"
echo
echo "4. To use Docker Compose (alternative to run script):"
echo "   cd $SCRIPT_DIR"
echo "   UID=\$(id -u) GID=\$(id -g) docker-compose up ros-jazzy"
echo
echo "Workspace: $HOST_WORKSPACE"
echo "Data directory: $HOST_DATA"
echo
echo "Total setup time: $SECONDS seconds"
echo