# RPI Camera Example Package for ROS Jazzy

This is an example package showing how to set up a camera node for ROS Jazzy on Raspberry Pi.

## Files

- `camera_publisher.py` - Simple camera publisher node
- `launch/camera_launch.py` - Launch file for camera setup
- `config/camera_params.yaml` - Camera configuration parameters

## Usage

1. Build the package:
   ```bash
   cd /opt/ros_ws
   colcon build --packages-select rpi_camera_example
   source install/setup.bash
   ```

2. Launch the camera:
   ```bash
   ros2 launch rpi_camera_example camera_launch.py
   ```

3. View the camera stream:
   ```bash
   ros2 run rqt_image_view rqt_image_view
   ```

## Configuration

Edit `config/camera_params.yaml` to adjust camera settings:
- Resolution (width, height)
- Frame rate
- Device path
- Frame ID

## Requirements

- Raspberry Pi with camera module
- ROS Jazzy with image transport packages
- OpenCV for Python (cv2)