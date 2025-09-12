# ROS Jazzy Workspace Examples

This directory is mounted into the Docker container at `/opt/ros_ws/src` and can contain your ROS packages.

## Getting Started

1. Create a new package:
   ```bash
   cd workspace
   ros2 pkg create --build-type ament_python my_robot_package
   ```

2. Build your workspace (from container):
   ```bash
   cd /opt/ros_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

## Example Packages

You can add example packages here or clone existing ones:

```bash
# Example: Clone a camera package
git clone https://github.com/ros-perception/image_transport_tutorials.git

# Example: Create a simple publisher/subscriber package
ros2 pkg create --build-type ament_python --dependencies rclpy sensor_msgs my_sensors
```

## Tips

- Use `--symlink-install` with colcon build for faster development
- Source your workspace after building: `source install/setup.bash`
- Use the ros_helper.sh script for common operations