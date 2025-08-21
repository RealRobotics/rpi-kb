# Docker Container with Ubuntu 24.04LTS and ROS Jazzy for Raspberry Pi 5

This guide explains how to set up a Docker container running Ubuntu 24.04LTS with ROS Jazzy on a Raspberry Pi 5, with full access to the Raspberry Pi camera.

## Overview

Running ROS Jazzy in a Docker container on Raspberry Pi 5 provides several advantages:

* **Isolation**: Keep your host system clean while experimenting with ROS
* **Portability**: Easy to backup, restore, and deploy across multiple Raspberry Pis
* **Consistency**: Ensures the same environment regardless of host OS version
* **Camera Access**: Full access to Raspberry Pi camera hardware from within the container

## Requirements

* Raspberry Pi 5 running the latest Raspberry Pi OS (64-bit)
* At least 8GB of RAM recommended
* 32GB+ microSD card (Class 10 or better)
* Raspberry Pi Camera Module (v2, v3, or compatible)
* Internet connection for initial setup

## Quick Start

For those who want to get started quickly, use the automated scripts:

```bash
# Clone this repository
git clone https://github.com/RealRobotics/rpi-kb.git
cd rpi-kb/ubuntu/docker_ros_jazzy

# Install Docker and dependencies
./install_docker.bash

# Build the ROS Jazzy container
./build_container.bash

# Run the container with camera access
./run_container.bash
```

## Manual Installation Process

### 1. Prepare the Raspberry Pi 5

Start with a fresh installation of Raspberry Pi OS (64-bit) on your Raspberry Pi 5. Ensure your system is up to date:

```bash
sudo apt update && sudo apt upgrade -y
sudo reboot
```

### 2. Install Docker

Docker is required to run containerized applications. The installation script will:
- Install Docker Engine
- Configure Docker to run without sudo
- Enable Docker service
- Install Docker Compose

Run the installation script:

```bash
./install_docker.bash
```

Or install manually:

```bash
# Install Docker's official GPG key
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add Docker repository
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker Engine
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Add user to docker group (requires logout/login to take effect)
sudo usermod -aG docker $USER

# Enable Docker service
sudo systemctl enable docker

# Reboot to ensure all changes take effect
sudo reboot
```

### 3. Configure Camera Access

For the Docker container to access the Raspberry Pi camera, we need to ensure proper device permissions and group membership:

```bash
# Add user to video group (if not already done)
sudo usermod -aG video $USER

# Create udev rule for camera access
echo 'SUBSYSTEM=="vchiq", GROUP="video", MODE="0660"' | sudo tee /etc/udev/rules.d/10-vchiq-permissions.rules
echo 'SUBSYSTEM=="dma_heap", GROUP="video", MODE="0660"' | sudo tee /etc/udev/rules.d/20-dma-heap.rules

# Reload udev rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Enable camera interface
sudo raspi-config nonint do_camera 0
```

### 4. Build the Docker Container

The container includes:
- Ubuntu 24.04LTS base image
- ROS 2 Jazzy Jalopy
- Essential development tools
- Camera utilities (libcamera, v4l-utils)
- Common ROS packages and dependencies

Build the container using the provided script:

```bash
./build_container.bash
```

This script will create a Docker image called `rpi-ros-jazzy` with all necessary components.

### 5. Run the Container

The run script starts the container with:
- Camera device access (`/dev/video*`)
- Video group permissions
- Shared volumes for persistent data
- X11 forwarding for GUI applications
- Network access

```bash
./run_container.bash
```

### 6. Test Camera Access

Once inside the container, test camera functionality:

```bash
# List available cameras
v4l2-ctl --list-devices

# Test camera with libcamera
libcamera-hello --timeout 5000

# Test with ROS 2
ros2 run image_tools cam2image
```

## Container Features

### Pre-installed ROS 2 Packages

The container includes commonly used ROS 2 packages:

* `ros-jazzy-desktop` - Full desktop installation
* `ros-jazzy-camera-ros` - Camera drivers and utilities
* `ros-jazzy-image-tools` - Image processing tools
* `ros-jazzy-cv-bridge` - OpenCV bridge
* `ros-jazzy-sensor-msgs` - Sensor message types
* `ros-jazzy-geometry-msgs` - Geometry message types

### Development Tools

* GCC/G++ compiler toolchain
* CMake and build-essential
* Git version control
* Python 3 with pip
* nano and vim text editors
* htop system monitor

### Camera Utilities

* `libcamera-apps` - Raspberry Pi camera applications
* `v4l-utils` - Video4Linux utilities
* OpenCV with Python bindings
* GStreamer multimedia framework

## Usage Examples

### Basic Camera Node

Create a simple camera publisher:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = CameraPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Launch File Example

Create a launch file for camera node:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_tools',
            executable='cam2image',
            name='camera_node',
            parameters=[
                {'width': 640},
                {'height': 480},
                {'frequency': 30.0}
            ]
        )
    ])
```

## Persistence and Data Management

### Shared Volumes

The container uses shared volumes to persist data:

* `~/ros2_ws` - ROS 2 workspace (mapped to host)
* `~/data` - General data storage
* `/tmp/.X11-unix` - X11 socket for GUI apps

### Backup and Restore

To backup your container and data:

```bash
# Save container state
docker commit rpi-ros-jazzy rpi-ros-jazzy:backup-$(date +%Y%m%d)

# Backup workspace
tar -czf ~/ros2_workspace_backup.tar.gz ~/ros2_ws

# List saved images
docker images rpi-ros-jazzy
```

## Troubleshooting

### Camera Not Detected

1. Check camera connection and enable it:
   ```bash
   sudo raspi-config nonint do_camera 0
   ```

2. Verify camera detection on host:
   ```bash
   lsusb  # For USB cameras
   vcgencmd get_camera  # For CSI cameras
   ```

3. Check device permissions:
   ```bash
   ls -la /dev/video*
   groups $USER  # Should include 'video'
   ```

### Container Issues

1. Docker permission denied:
   ```bash
   sudo usermod -aG docker $USER
   # Log out and back in
   ```

2. Out of memory during build:
   ```bash
   # Increase swap file size
   sudo dphys-swapfile swapoff
   sudo sed -i 's/CONF_SWAPSIZE=100/CONF_SWAPSIZE=2048/' /etc/dphys-swapfile
   sudo dphys-swapfile setup
   sudo dphys-swapfile swapon
   ```

3. X11 forwarding not working:
   ```bash
   xhost +local:docker
   ```

### Performance Optimization

1. Use multi-stage builds to reduce image size
2. Enable Docker's experimental features for better ARM64 support
3. Use tmpfs for temporary files
4. Optimize ROS 2 DDS settings for single-board computers

## Advanced Configuration

### Custom ROS 2 Packages

To add your own packages to the container:

1. Create a `packages.txt` file with package names
2. Modify the Dockerfile to install them
3. Rebuild the container

### GPU Acceleration

For hardware-accelerated computer vision:

```bash
# Add GPU support to container
docker run --device /dev/dri --device /dev/vchiq ...
```

### Network Configuration

For multi-robot setups:

```bash
# Set ROS_DOMAIN_ID for isolation
export ROS_DOMAIN_ID=42

# Configure DDS settings
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
```

## References

* [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
* [Docker Documentation](https://docs.docker.com/)
* [Raspberry Pi Camera Documentation](https://www.raspberrypi.org/documentation/usage/camera/)
* [libcamera Documentation](https://libcamera.org/docs.html)

## Changelog

* **v1.0** - Initial release with Ubuntu 24.04LTS and ROS Jazzy support