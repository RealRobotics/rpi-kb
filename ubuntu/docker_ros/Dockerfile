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
RUN mkdir -p /home/ubuntu/ws/src
WORKDIR /home/ubuntu/ws

# Prepare to change to user ubuntu (already exists in base image with uid and gid = 1000).
# Change ownership of home directory to ubuntu:ubuntu.
RUN chown -R ubuntu:ubuntu /home/ubuntu/

# Set home directory
ENV HOME=/home/ubuntu

# Use display
ENV DISPLAY=:1.0

# Change to user ubuntu.
USER ubuntu

# Setup environment
RUN echo ". /opt/ros/foxy/setup.bash" > /home/ubuntu/.bash_aliases
RUN echo "source /home/ubuntu/ws/install/setup.bash" >> /home/ubuntu/.bash_aliases
