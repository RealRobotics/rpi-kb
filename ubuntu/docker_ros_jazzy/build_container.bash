#!/bin/bash
# Build Docker container with Ubuntu 24.04LTS and ROS 2 Jazzy

# Stop on first error.
set -e

# Tell the user what is going on.
echo
echo "Running $0..."
echo "Building Docker container with Ubuntu 24.04LTS and ROS 2 Jazzy"
echo

# Check if Docker is installed and running
if ! command -v docker >/dev/null 2>&1; then
    echo "Error: Docker is not installed. Please run ./install_docker.bash first."
    exit 1
fi

if ! docker info >/dev/null 2>&1; then
    echo "Error: Docker daemon is not running or you don't have permission."
    echo "Make sure Docker is running and you're in the docker group."
    echo "You may need to log out and back in after running install_docker.bash"
    exit 1
fi

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Set image name and tag
IMAGE_NAME="rpi-ros-jazzy"
IMAGE_TAG="latest"
FULL_IMAGE_NAME="${IMAGE_NAME}:${IMAGE_TAG}"

echo "Building Docker image: $FULL_IMAGE_NAME"
echo "Build context: $SCRIPT_DIR"
echo

# Build the Docker image
# Use --no-cache if you want to force a complete rebuild
# Add --progress=plain for more verbose output
docker build \
    --tag "$FULL_IMAGE_NAME" \
    --file "$SCRIPT_DIR/Dockerfile" \
    --build-arg USERNAME="$(whoami)" \
    --build-arg USER_UID="$(id -u)" \
    --build-arg USER_GID="$(id -g)" \
    "$SCRIPT_DIR"

# Check if build was successful
if [ $? -eq 0 ]; then
    echo
    echo "✓ Docker image built successfully!"
    echo
    echo "Image details:"
    docker images "$IMAGE_NAME"
    echo
    echo "To run the container, use:"
    echo "  ./run_container.bash"
    echo
    echo "Or run manually with:"
    echo "  docker run -it --rm $FULL_IMAGE_NAME"
else
    echo
    echo "✗ Docker build failed!"
    echo "Please check the error messages above."
    exit 1
fi

echo
echo "$0 took $SECONDS seconds."
echo