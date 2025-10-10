# Use the Raspberry Pi Camera 3 on a Raspberry Pi 5 running Ubuntu 24.04LTS

We need to build a bunch of code to make this work.

## Install Ubuntu 24.04LTS

First, install Ubuntu 24.04LTS for a Raspberry Pi 5 using the  [Raspberry Pi Imager](https://www.raspberrypi.com/software/).

Once you have gone through the first time configuration screens, try to open a terminal shell. If it fails, switch to a full screen terminal, `Ctrl+Alt F3`, login and then execute this command:

```bash
sudo apt-get remove xdg-desktop-portal-gnome
```

Exit the terminal and return to the graphics screen, `Ctrl+Alt F1`, login and try again. This time it should open correctly.

## Install libcamera apps and ROS camera app

Clone this repo on the Raspberry Pi.  Then download, build and install the code using the script `install_libcamera.bash`.  If you want to use ROS2 as well, then run the script `install_camera_ros.bash`.

## References

Based on: <https://github.com/TanmayChhatbar/ros2_car/blob/feature/camera_setup/README_ELI5.md>.
Calls out to: <https://www.raspberrypi.com/documentation/computers/camera_software.html#building-libcamera>
and: <https://github.com/christianrauch/camera_ros#build-instructions>

## Last updated

10 October 2025
