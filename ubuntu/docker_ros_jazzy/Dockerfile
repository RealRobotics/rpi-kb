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

# Setup environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /home/ubuntu/ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
