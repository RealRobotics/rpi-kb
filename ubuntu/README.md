# Ubuntu Installation

Ubuntu is the OS of choice when using ROS and ROS2. The Raspberry Pi can be used for both 64 and 32 bits installations. However, this document deals with installation of Ubuntu 64 bit OS as the ROS2 packages are pre-compiled and can be installed using `apt`.

ROS Jazzy tested on these releases:

* Ubuntu Desktop 24.04 LTS 64 bit.
* Ubuntu Server 24.04 LTS 64 bit.

For legacy support, ROS Humble tested on these releases:

* Ubuntu Desktop 22.04.1 LTS 64 bit.
* Ubuntu Server 22.04.1 LTS 64 bit.

## Installation

The Raspberry Pi imager tool is the best choice for installing images onto an SD card. Just make sure you choose the correct version of the OS.

For ROS Jazzy, use Ubuntu 24.04 LTS. For ROS Humble/Iron, use Ubuntu 22.04 LTS.

Once the image has been created, install the SD card in the RPi and then go through the configuration screens.   Select:

* English
* UK keyboard
* London as the time zone.
* Leave Wi-Fi until later.

The Ubuntu setup runs for a while and then you do some more configuration screen.  Skip all of these and then the system boots into the OS properly.

### Disable unattended upgrades

Unattended or automatic upgrades are great for always on IOT devices that are connected to the Internet so need to be patched regularly.  However, our robots are normally isolated from the internet so automatic upgrades just cause lots of inconvenience.

```bash
sudo nano /etc/apt/apt.conf.d/20auto-upgrades
```

After editing the contents should look like this:

```text
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";
```

Save and exit and reboot.

## Raspberry Pi Camera v2

The setup of the Raspberry Pi Camera Module for Ubuntu 22.04LTS is a bit of a pain as the transition to using `libcamera` had not been fully implemented in that Ubuntu release. However, Ubuntu 24.04LTS with ROS Jazzy has much better camera support out of the box. For Ubuntu 22.04LTS, you can build and install the Raspberry Pi specific `libcamera` code by running [this script](pi_camera/install_libcamera.bash).

To run the camera with ROS 2, I found this repo that works well enough for what I needed, so I forked it as I needed to add launch files and a README. <https://github.com/pipebots/camera_ros/tree/add-launch-files>.

## Raspberry Pi Camera v3

With ROS Jazzy and Ubuntu 24.04LTS, the camera v3 should work much better than with previous ROS versions. For ROS Humble or Iron on Ubuntu 22.04LTS, running the camera v3 makes things more difficult as the kernel has to be updated. The process has been documented [here](pi_camera/arducam.md).

## Docker Setup for ROS Jazzy

For a containerized ROS Jazzy setup, see the Docker scripts and documentation in [docker_ros_jazzy](docker_ros_jazzy/).
