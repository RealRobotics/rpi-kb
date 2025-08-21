#!/bin/bash
# Run Docker container with camera access and proper volume mounts

# Stop on first error.
set -e

# Tell the user what is going on.
echo
echo "Running $0..."
echo "Starting ROS 2 Jazzy container with camera access"
echo

# Check if Docker is installed and running
if ! command -v docker >/dev/null 2>&1; then
    echo "Error: Docker is not installed. Please run ./install_docker.bash first."
    exit 1
fi

if ! docker info >/dev/null 2>&1; then
    echo "Error: Docker daemon is not running or you don't have permission."
    echo "Make sure Docker is running and you're in the docker group."
    exit 1
fi

# Set image name
IMAGE_NAME="rpi-ros-jazzy:latest"

# Check if image exists
if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
    echo "Error: Docker image '$IMAGE_NAME' not found."
    echo "Please run ./build_container.bash first."
    exit 1
fi

# Set container name
CONTAINER_NAME="ros-jazzy-container"

# Stop and remove existing container if it exists
if docker ps -a --format 'table {{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Stopping and removing existing container: $CONTAINER_NAME"
    docker stop "$CONTAINER_NAME" >/dev/null 2>&1 || true
    docker rm "$CONTAINER_NAME" >/dev/null 2>&1 || true
fi

# Create host directories for persistent data
HOST_WORKSPACE="$HOME/ros2_ws"
HOST_DATA="$HOME/docker_data"

mkdir -p "$HOST_WORKSPACE/src"
mkdir -p "$HOST_DATA"

echo "Host workspace: $HOST_WORKSPACE"
echo "Host data directory: $HOST_DATA"

# Detect available camera devices
CAMERA_DEVICES=""
for device in /dev/video*; do
    if [ -e "$device" ]; then
        CAMERA_DEVICES="$CAMERA_DEVICES --device=$device"
        echo "Found camera device: $device"
    fi
done

# Add other potential camera-related devices
for device in /dev/vchiq /dev/vcsm /dev/vcsm-cma; do
    if [ -e "$device" ]; then
        CAMERA_DEVICES="$CAMERA_DEVICES --device=$device"
        echo "Found device: $device"
    fi
done

if [ -z "$CAMERA_DEVICES" ]; then
    echo "Warning: No camera devices found. Camera functionality may not work."
    echo "Make sure your camera is connected and enabled."
fi

# Set up X11 forwarding for GUI applications
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth

# Create xauth file if it doesn't exist
if [ ! -f $XAUTH ]; then
    touch $XAUTH
    chmod 666 $XAUTH
fi

# Add current display to xauth
if [ -n "$DISPLAY" ]; then
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
fi

echo "Starting container with the following configuration:"
echo "  - Container name: $CONTAINER_NAME"
echo "  - Image: $IMAGE_NAME"
echo "  - Workspace mount: $HOST_WORKSPACE -> /home/$(whoami)/ros2_ws"
echo "  - Data mount: $HOST_DATA -> /home/$(whoami)/data"
echo "  - Camera devices: $CAMERA_DEVICES"
echo "  - X11 forwarding: enabled"
echo

# Run the container
docker run -it \
    --name "$CONTAINER_NAME" \
    --hostname "ros-jazzy-pi" \
    --privileged \
    --network host \
    --env DISPLAY="$DISPLAY" \
    --env XAUTHORITY="$XAUTH" \
    --volume "$XSOCK:$XSOCK:rw" \
    --volume "$XAUTH:$XAUTH:rw" \
    --volume "$HOST_WORKSPACE:/home/$(whoami)/ros2_ws:rw" \
    --volume "$HOST_DATA:/home/$(whoami)/data:rw" \
    --volume "/dev/shm:/dev/shm:rw" \
    --group-add video \
    --group-add audio \
    $CAMERA_DEVICES \
    "$IMAGE_NAME"

echo
echo "Container has exited."
echo "To remove the stopped container, run:"
echo "  docker rm $CONTAINER_NAME"
echo
echo "$0 took $SECONDS seconds."
echo