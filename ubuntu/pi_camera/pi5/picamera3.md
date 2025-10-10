# Use the Raspberry Pi Camera 3 on a Raspberry Pi 5 running Ubuntu 24.04LTS

We need to build a bunch of code.

## Quick install

Download, build and install the code using the script `install_libcamera.bash`.  If you want to use ROS2 as well, then run the script `install_camera_ros.bash`.

### The full method

Started here: <https://github.com/TanmayChhatbar/ros2_car/blob/feature/camera_setup/README_ELI5.md>.  This says:

1. Build `libcamera` and `rpicam-apps` from source following this [guide](https://www.raspberrypi.com/documentation/computers/camera_software.html#building-libcamera).  We need to do this as the `libcamera` libraries that are installed through `apt` does not detect the Pi Cameras.
   1.
