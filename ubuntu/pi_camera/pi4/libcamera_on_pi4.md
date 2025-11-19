# Libcamera on Pi4 using Ubuntu

## Raspberry Pi Camera v2

The setup of the Raspberry Pi Camera Module for Ubuntu 22.04LTS is a bit of a pain as the transition to using `libcamera` had not been fully implemented in this Ubuntu release.  I spent many, many hours trying different ways to make this work and in the end it was quite simple once you realise that you have to build and install the Raspberry Pi specific `libcamera` code.  To build and install the Raspberry Pi `libcamera` libraries and apps, run [this script](install_libcamera.bash).

To run the camera with ROS 2, I found this repo that works well enough for what I needed, so I forked it as I needed to add launch files and a README. <https://github.com/pipebots/camera_ros/tree/add-launch-files>.

## Raspberry Pi Camera v3

Running the camera v3 with ROS2 Humble or Iron makes things a lot more difficult as the kernel has to be updated.  The process has been documented [here](pi_camera/arducam.md).
