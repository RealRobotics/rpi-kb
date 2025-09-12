# ROS Jazzy Docker setup for Raspberry Pi
FROM ros:jazzy-desktop

# Set environment variables
ENV ROS_DISTRO=jazzy
ENV DEBIAN_FRONTEND=noninteractive

# Install additional packages commonly needed for robotics
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libgoogle-glog-dev \
    libgtest-dev \
    nano \
    python3-pip \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    ros-jazzy-desktop \
    ros-jazzy-rqt* \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Create a workspace
RUN mkdir -p /opt/ros_ws/src
WORKDIR /opt/ros_ws

# Source ROS setup in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /opt/ros_ws/devel/setup.bash" >> /root/.bashrc

# Set up entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]