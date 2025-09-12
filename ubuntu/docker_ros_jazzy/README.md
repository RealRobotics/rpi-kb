# ROS Jazzy Docker Setup for Raspberry Pi

This directory contains Docker setup files for running ROS Jazzy on Raspberry Pi in a containerized environment.

## Files

- `Dockerfile` - Docker image definition based on ros:jazzy-desktop
- `docker-compose.yml` - Docker Compose configuration for easier container management
- `entrypoint.sh` - Container entrypoint script that sources ROS environment
- `setup_docker.sh` - Setup script to build the Docker image
- `run_container.sh` - Script to run the container with appropriate settings
- `workspace/` - Directory for ROS packages (mounted into container)

## Prerequisites

1. Docker installed on your Raspberry Pi:
   ```bash
   curl -fsSL https://get.docker.com -o get-docker.sh
   sudo sh get-docker.sh
   sudo usermod -aG docker $USER
   # Log out and back in for group changes to take effect
   ```

2. Docker Compose (usually comes with Docker):
   ```bash
   # Test if available
   docker compose version
   ```

## Quick Start

1. Build the ROS Jazzy Docker image:
   ```bash
   ./setup_docker.sh
   ```

2. Run the container:
   ```bash
   ./run_container.sh
   ```

   Or using Docker Compose:
   ```bash
   docker-compose up -d
   docker-compose exec ros_jazzy bash
   ```

## Container Features

- Based on official `ros:jazzy-desktop` image
- Ubuntu 24.04 LTS with ROS Jazzy
- Common robotics packages pre-installed
- GUI applications support (X11 forwarding)
- Camera access (if /dev/video0 exists)
- Host networking for ROS communication
- Workspace directory mounted at `/opt/ros_ws/src`

## Environment Variables

- `ROS_DOMAIN_ID` - ROS 2 domain ID (default: 0)
- `DISPLAY` - X11 display for GUI applications

## Usage Examples

### Basic ROS Commands
```bash
# Inside the container
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_py talker
```

### Create a Workspace Package
```bash
# Inside the container, in /opt/ros_ws
cd src
ros2 pkg create --build-type ament_python my_package
cd ..
colcon build
source install/setup.bash
```

### Run with Camera
The container automatically tries to mount `/dev/video0` for camera access. Make sure your user has permission to access video devices:
```bash
sudo usermod -a -G video $USER
```

## Troubleshooting

### GUI Applications
If GUI applications don't work, try:
```bash
xhost +local:docker
```

### Permission Issues
If you have permission issues with mounted volumes:
```bash
sudo chown -R $USER:$USER workspace/
```

### Camera Not Working
Check if the camera is available:
```bash
ls -la /dev/video*
```

## Environment Sourcing

The container automatically sources:
1. `/opt/ros/jazzy/setup.bash` - ROS Jazzy environment
2. `/opt/ros_ws/devel/setup.bash` - Workspace environment (if exists)

These are sourced in the entrypoint script and also added to `.bashrc` for interactive sessions.