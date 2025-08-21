# Real Robotics Raspberry Pi knowledge base

This repo contains information on how to set up Raspberry Pis for work with robotics and measurement platforms at the University of Leeds.

- [Real Robotics Raspberry Pi knowledge base](#real-robotics-raspberry-pi-knowledge-base)
  - [SD card images](#sd-card-images)
    - [Back up and restore an image](#back-up-and-restore-an-image)
    - [Customising Ubuntu 22.04LTS for the Raspberry Pi](#customising-ubuntu-2204lts-for-the-raspberry-pi)
  - [Connect to Eduroam](#connect-to-eduroam)
  - [Libcamera on Ubuntu 22.04LTS](#libcamera-on-ubuntu-2204lts)
  - [Docker with Ubuntu 24.04LTS and ROS Jazzy](#docker-with-ubuntu-2404lts-and-ros-jazzy)
  - [Networking](#networking)
  - [Acknowledgments](#acknowledgments)

## SD card images

Raspberry Pi provides a nice imager tool to download and create an SD card image for the operating system (OS) along with great instructions, so use them! [See here for details.](https://www.raspberrypi.com/software/)

For robotics use, the preferred OS is Ubuntu 20.04 or 22.04, 64 bit.  This is best if you use ROS as the packages are prebuilt for the Pi so can be installed without building from source.

For general purpose use, the Raspberry Pi OS with desktop (32 bit) is best. This release has all the recommended software and has the best online support.

The micro SD card should be as fast as possible and 16GB or larger.  My personal preference is the 32GB SanDisk Extreme micro SD cards.  You can get them from [Amazon](https://www.amazon.co.uk/SanDisk-Extreme-microSDHC-Adapter-Performance/dp/B06XWMQ81P/ref=asc_df_B06XWMQ81P/) or Science Warehouse.

### Back up and restore an image

Once you have prepared an SD card, it is useful to be able to create a back up copy and restore the copy when needed.  This process is documented [here](backup/backup.md).

### Customising Ubuntu 22.04LTS for the Raspberry Pi

There are some useful tricks and tips for setting up Ubuntu for robotics use [here](ubuntu/README.md).  There are also some notes on using `libcamera` on the Raspberry Pi (22.04LTS does not have this working so you have to do a fair bit manually).

## Connect to Eduroam

At home, it is easy to connect a Raspberry Pi to the internet using your home Wi-Fi.  Eduroam is the Wi-Fi network for the university so it is common for Raspberry Pis to be connected to Eduroam at some point.

__However, due to the often lax security on Raspberry Pis caused by poor passwords etc., it is recommended that the Raspberry Pis are only connected to Eduroam during updates etc. and are disconnected afterwards.__

The documentation for connecting to Eduroam is [here](eduroam/eduroam.md).

## Libcamera on Ubuntu 22.04LTS

ROS Humble and Iron both use Ubuntu 22.04LTS as their OS.  Unfortunately, the RPi camera v3 needs a later version of the Debian OS and kernel.  This makes things messy!
To install `libcamera` and the necessary tools, please see this [README](ubuntu/README.md).

## Docker with Ubuntu 24.04LTS and ROS Jazzy

For the latest ROS 2 Jazzy release, you can run Ubuntu 24.04LTS in a Docker container on your Raspberry Pi 5. This provides an isolated environment with full camera access and the latest ROS 2 features. See the [Docker setup guide](ubuntu/docker_ros_jazzy/README.md) for complete instructions and automated scripts.

## Networking

This section is not Raspberry Pi specific but might help someone to use a robot using a Raspberry Pi in difficult circumstances.

* [Networking in buried pipes.](networking/BuriedPipeNetworkSetup.md)

## Acknowledgments

&copy; 2022-2023, University of Leeds.

The author, A. Blight, has asserted his moral rights.
