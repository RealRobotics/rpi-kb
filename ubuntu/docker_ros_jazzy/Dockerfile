# Docker image for Raspberry Pi 5 with Ubuntu 24.04LTS and ROS 2 Jazzy
FROM ubuntu:24.04

# Set non-interactive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# Set locale
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Set ROS 2 environment variables
ENV ROS_DISTRO=jazzy
ENV ROS_VERSION=2
ENV ROS_PYTHON_VERSION=3

# Create a non-root user
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install system dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 official repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package list with ROS 2 repository
RUN apt-get update

# Install ROS 2 Jazzy Desktop
RUN apt-get install -y \
    ros-jazzy-desktop \
    ros-jazzy-camera-ros \
    ros-jazzy-image-tools \
    ros-jazzy-cv-bridge \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-rclcpp \
    ros-jazzy-rclpy \
    ros-jazzy-launch \
    ros-jazzy-launch-ros \
    && rm -rf /var/lib/apt/lists/*

# Install development tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    nano \
    vim \
    htop \
    tree \
    && rm -rf /var/lib/apt/lists/*

# Install camera and multimedia libraries
RUN apt-get update && apt-get install -y \
    libcamera-apps \
    libcamera-dev \
    libcamera-tools \
    v4l-utils \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    python3-opencv \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# Install additional useful packages
RUN apt-get update && apt-get install -y \
    tmux \
    screen \
    curl \
    wget \
    unzip \
    zip \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Set up workspace directory
RUN mkdir -p /home/$USERNAME/ros2_ws/src
RUN chown -R $USERNAME:$USERNAME /home/$USERNAME/ros2_ws

# Switch to non-root user
USER $USERNAME

# Set up ROS 2 environment in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /home/$USERNAME/ros2_ws/install/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "export ROS_DOMAIN_ID=0" >> /home/$USERNAME/.bashrc

# Set working directory
WORKDIR /home/$USERNAME/ros2_ws

# Create a simple test node
RUN mkdir -p /home/$USERNAME/ros2_ws/src/camera_test
COPY --chown=$USERNAME:$USERNAME camera_test_node.py /home/$USERNAME/ros2_ws/src/camera_test/
RUN echo "#!/usr/bin/env python3" > /home/$USERNAME/ros2_ws/src/camera_test/camera_test_node.py \
    && echo "import rclpy" >> /home/$USERNAME/ros2_ws/src/camera_test/camera_test_node.py \
    && echo "from rclpy.node import Node" >> /home/$USERNAME/ros2_ws/src/camera_test/camera_test_node.py \
    && echo "from sensor_msgs.msg import Image" >> /home/$USERNAME/ros2_ws/src/camera_test/camera_test_node.py \
    && echo "" >> /home/$USERNAME/ros2_ws/src/camera_test/camera_test_node.py \
    && echo "class CameraTestNode(Node):" >> /home/$USERNAME/ros2_ws/src/camera_test/camera_test_node.py \
    && echo "    def __init__(self):" >> /home/$USERNAME/ros2_ws/src/camera_test/camera_test_node.py \
    && echo "        super().__init__('camera_test')" >> /home/$USERNAME/ros2_ws/src/camera_test/camera_test_node.py \
    && echo "        self.get_logger().info('Camera test node started')" >> /home/$USERNAME/ros2_ws/src/camera_test/camera_test_node.py \
    && echo "" >> /home/$USERNAME/ros2_ws/src/camera_test/camera_test_node.py \
    && echo "def main():" >> /home/$USERNAME/ros2_ws/src/camera_test/camera_test_node.py \
    && echo "    rclpy.init()" >> /home/$USERNAME/ros2_ws/src/camera_test/camera_test_node.py \
    && echo "    node = CameraTestNode()" >> /home/$USERNAME/ros2_ws/src/camera_test/camera_test_node.py \
    && echo "    rclpy.spin(node)" >> /home/$USERNAME/ros2_ws/src/camera_test/camera_test_node.py \
    && echo "    rclpy.shutdown()" >> /home/$USERNAME/ros2_ws/src/camera_test/camera_test_node.py \
    && echo "" >> /home/$USERNAME/ros2_ws/src/camera_test/camera_test_node.py \
    && echo "if __name__ == '__main__':" >> /home/$USERNAME/ros2_ws/src/camera_test/camera_test_node.py \
    && echo "    main()" >> /home/$USERNAME/ros2_ws/src/camera_test/camera_test_node.py

# Set entrypoint to bash with ROS 2 environment loaded
CMD ["/bin/bash", "-l"]