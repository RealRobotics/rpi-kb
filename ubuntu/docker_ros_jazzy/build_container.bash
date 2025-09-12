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
    echo "Creating Dockerfile for ROS development..."
    cat << 'EOF' > Dockerfile
FROM ros:jazzy-perception

# Install additional packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    nano \
    git \
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep update

# Create workspace
RUN mkdir -p /ws/src
WORKDIR /ws

# Setup environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
EOF
fi

# Build the container
echo "Building robotics development container..."
docker build -t rpi-robotics:latest .

echo
echo "Container built successfully! You can run it with:"
echo "docker run -it --rm rpi-robotics:latest"
echo
