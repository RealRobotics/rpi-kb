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

# Create a simple Dockerfile for ROS development if it doesn't exist
if [ ! -f "Dockerfile" ]; then
    echo "Error: Dockerfile not found!"
    exit 1
fi

# Build the container
echo "Building robotics development container..."
docker build -t rpi-robotics:latest .

echo
echo "Container built successfully! You can run it with:"
echo "docker run -it --rm rpi-robotics:latest"
echo
