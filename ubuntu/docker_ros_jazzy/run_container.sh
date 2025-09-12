#!/bin/bash
# Script to run the ROS Jazzy Docker container

# Colors for output
GREEN='\033[0;32m'
NC='\033[0m' # No Color

# Check if the image exists
if ! docker image inspect rpi-ros-jazzy:latest &> /dev/null; then
    echo "ROS Jazzy Docker image not found. Building it now..."
    ./setup_docker.sh
fi

# Create workspace directory if it doesn't exist
if [ ! -d "workspace" ]; then
    mkdir -p workspace
fi

echo -e "${GREEN}Starting ROS Jazzy container...${NC}"

# Run the container with appropriate settings
docker run -it --rm \
    --name ros_jazzy_container \
    --network host \
    --privileged \
    -e DISPLAY=${DISPLAY:-:0} \
    -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$(pwd)/workspace:/opt/ros_ws/src" \
    -v /dev:/dev \
    --device /dev/video0:/dev/video0 2>/dev/null || true \
    -w /opt/ros_ws \
    rpi-ros-jazzy:latest