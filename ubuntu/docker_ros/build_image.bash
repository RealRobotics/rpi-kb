#!/bin/bash
# Build Docker container for robotics development

# Stop on first error
set -e

# Tell the user what is going on
echo
echo "Building Docker container..."
echo

# Check if Docker is available and user can run docker commands
if ! docker info &>/dev/null; then
    echo "Error: Cannot connect to Docker daemon. Make sure Docker is running and you have proper permissions."
    echo "If you just installed Docker, you may need to log out and log back in."
    exit 1
fi
echo "Docker access confirmed!"

# Create a simple Dockerfile for ROS development if it doesn't exist
if [ ! -f "Dockerfile" ]; then
    echo "Error: Dockerfile not found!"
    exit 1
fi

# Build the container
echo "Building robotics development container..."
docker build -t rpi-robotics:latest .

# Setup X11 forwarding using modular script
if [ -f "./setup_x11.bash" ]; then
    ./setup_x11.bash
else
    echo "Warning: setup_x11.bash not found. Skipping X11 setup."
fi

echo
echo "Testing camera functionality..."
# Test camera using modular script
if [ -f "./test_camera.bash" ]; then
    chmod +x ./test_camera.bash
    if ./test_camera.bash; then
        echo "Camera test completed successfully!"
    else
        echo "Camera test failed or no camera detected."
    fi
else
    echo "Warning: test_camera.bash not found. Skipping camera test."
    echo "Camera support scripts are available in ./ubuntu/pi_camera/"
fi


./create_examples.bash

echo
echo "Container built successfully! You can run it with:"
echo "docker run -it --rm rpi-robotics:latest"
echo "or use the provided start.bash and attach.bash scripts."
echo
