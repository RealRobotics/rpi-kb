# Use the Raspberry Pi Camera 3 on a Raspberry Pi 5 running Ubuntu 24.04LTS

We need to build a bunch of code to make this work.

## Install Ubuntu 24.04LTS

First, install Ubuntu 24.04LTS for a Raspberry Pi 5 using the  [Raspberry Pi Imager](https://www.raspberrypi.com/software/).

Once you have gone through the first time configuration screens, try to open a terminal shell. If it fails, switch to a full screen terminal, `Ctrl+Alt F3`, login and then execute this command:

```bash
sudo apt-get remove xdg-desktop-portal-gnome
```

Exit the terminal and return to the graphics screen, `Ctrl+Alt F1`, login and try again. This time it should open correctly.

## Install libcamera apps

Clone this repo on the Raspberry Pi.

```bash
mkdir -p ~/git
cd git
git clone https://github.com/RealRobotics/rpi-kb.git
```

Then download, build and install the code using the script `install_libcamera.bash`.

Test by running the app `qcam` which should find the camera and display an image.

## Install the camera_ros package

If you have previously installed ROS, you can just run the script `install_camera_ros.bash` to build and install the `camera_ros` package.

If you want to build this in an existing workspace, modify the `WORKSPACE_NAME` variable at the top of the file `install_camera_ros.bash` as needed.

## References

Based on: <https://github.com/TanmayChhatbar/ros2_car/blob/feature/camera_setup/README_ELI5.md>.
Calls out to: <https://www.raspberrypi.com/documentation/computers/camera_software.html#building-libcamera>
and: <https://github.com/christianrauch/camera_ros#build-instructions>

## Last updated

20 November 2025
