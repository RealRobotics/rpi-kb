# Quick Reference: Docker ROS Jazzy on Raspberry Pi 5

## One-Line Setup

For the impatient, run this single command to set up everything:

```bash
git clone https://github.com/RealRobotics/rpi-kb.git && cd rpi-kb/ubuntu/docker_ros_jazzy && ./setup.bash
```

## Individual Scripts

| Script | Purpose | Usage |
|--------|---------|-------|
| `setup.bash` | Complete automated setup | `./setup.bash` |
| `install_docker.bash` | Install Docker only | `./install_docker.bash` |
| `build_container.bash` | Build ROS Jazzy container | `./build_container.bash` |
| `run_container.bash` | Start container with camera access | `./run_container.bash` |
| `test_camera.bash` | Test camera functionality (run inside container) | `./test_camera.bash` |

## Quick Commands

### Start Container
```bash
./run_container.bash
```

### Using Docker Compose
```bash
UID=$(id -u) GID=$(id -g) docker-compose up ros-jazzy
```

### Test Camera (inside container)
```bash
# List cameras
v4l2-ctl --list-devices

# Test with libcamera
libcamera-hello --timeout 5000

# Test with ROS 2
ros2 run image_tools cam2image
```

### ROS 2 Camera Commands
```bash
# Start camera publisher
ros2 run image_tools cam2image

# List topics
ros2 topic list

# View image data
ros2 topic echo /image

# Run custom camera node
python3 camera_publisher.py
```

## File Overview

- `Dockerfile` - Ubuntu 24.04LTS + ROS Jazzy container definition
- `docker-compose.yml` - Multi-service container orchestration
- `camera_publisher.py` - Example ROS 2 camera node
- `camera_launch.py` - Launch file for camera setup
- `README.md` - Complete documentation

## Troubleshooting

### Camera not detected
```bash
# Check devices
ls -la /dev/video*

# Check permissions
groups $USER  # Should include 'video'

# Enable camera (on host)
sudo raspi-config nonint do_camera 0
```

### Docker permission denied
```bash
# Add user to docker group (requires logout/login)
sudo usermod -aG docker $USER
```

### Container build fails
```bash
# Increase swap (if low memory)
sudo dphys-swapfile swapoff
sudo sed -i 's/CONF_SWAPSIZE=100/CONF_SWAPSIZE=2048/' /etc/dphys-swapfile
sudo dphys-swapfile setup
sudo dphys-swapfile swapon
```

## System Requirements

- Raspberry Pi 5
- Raspberry Pi OS (64-bit)
- 8GB+ RAM recommended
- 32GB+ microSD card
- Raspberry Pi Camera Module

## What's Included

- Ubuntu 24.04LTS base
- ROS 2 Jazzy Desktop
- Camera support (libcamera, v4l-utils)
- Development tools (GCC, CMake, Git)
- Example camera nodes
- X11 forwarding for GUI apps
- Persistent workspace volumes