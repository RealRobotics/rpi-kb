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
    sudo

# Camera support
RUN apt-get update && apt-get install -y \
    libcamera-tools \
    libcamera-ipa \
    v4l-utils \
    ros-jazzy-camera-ros

# Clean up to save space.
# RUN apt-get clean \
#     && rm -rf /var/lib/apt/lists/*

# Set the password to "rr1234" for user ubuntu (for sudo access if needed).
RUN echo "ubuntu:rr1234" | chpasswd

# Create workspace
RUN mkdir -p /home/ubuntu/ws/src
WORKDIR /home/ubuntu/ws

# Prepare to change to user ubuntu (already exists in base image with uid and gid = 1000).
# Change ownership of home directory to ubuntu:ubuntu.
RUN chown -R ubuntu:ubuntu /home/ubuntu/
# Add ubuntu user to video group so it can access camera devices inside container
RUN usermod -aG video ubuntu || true
# Set home directory
ENV HOME=/home/ubuntu

# Use display
ENV DISPLAY=:1.0

# Change to user ubuntu.
USER ubuntu

# Setup ROS environment
RUN echo ". /opt/ros/jazzy/setup.bash" > /home/ubuntu/.bash_aliases
RUN echo "source /home/ubuntu/ws/install/setup.bash" >> /home/ubuntu/.bash_aliases

# Initialize rosdep
RUN rosdep update
