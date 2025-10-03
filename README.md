# Real Robotics Raspberry Pi knowledge base

This repo contains information on how to set up Raspberry Pis for work with robotics and measurement platforms at the University of Leeds.

- [Real Robotics Raspberry Pi knowledge base](#real-robotics-raspberry-pi-knowledge-base)
  - [SD card images](#sd-card-images)
    - [Back up and restore an image](#back-up-and-restore-an-image)
  - [Connect to Eduroam](#connect-to-eduroam)
  - [Ubuntu](#ubuntu)
    - [Libcamera on Ubuntu 22.04LTS](#libcamera-on-ubuntu-2204lts)
    - [Raspberry Pi 5, Ubuntu 24.04LTS and cameras](#raspberry-pi-5-ubuntu-2404lts-and-cameras)
  - [Networking](#networking)
  - [Acknowledgments](#acknowledgments)

## SD card images

Raspberry Pi provides a nice imager tool to download and create an SD card image for the operating system (OS) along with great instructions, so use them! [See here for details.](https://www.raspberrypi.com/software/)

For robotics use, the preferred OS is Ubuntu 20.04 or 22.04, 64 bit.  This is best if you use ROS as the packages are prebuilt for the Pi so can be installed without building from source.

For general purpose use, the Raspberry Pi OS with desktop (32 bit) is best. This release has all the recommended software and has the best online support.

The micro SD card should be as fast as possible and 16GB or larger.  My personal preference is the 32GB SanDisk Extreme micro SD cards.  You can get them from [Amazon](https://www.amazon.co.uk/SanDisk-Extreme-microSDHC-Adapter-Performance/dp/B06XWMQ81P/ref=asc_df_B06XWMQ81P/) or Science Warehouse.

### Back up and restore an image

Once you have prepared an SD card, it is useful to be able to create a back up copy and restore the copy when needed.  This process is documented [here](backup/backup.md).

## Connect to Eduroam

At home, it is easy to connect a Raspberry Pi to the internet using your home Wi-Fi.  Eduroam is the Wi-Fi network for the university so it is common for Raspberry Pis to be connected to Eduroam at some point.

__However, due to the often lax security on Raspberry Pis caused by poor passwords etc., it is recommended that the Raspberry Pis are only connected to Eduroam during updates etc. and are disconnected afterwards.__

The documentation for connecting to Eduroam is [here](eduroam/eduroam.md).

## Ubuntu

### Libcamera on Ubuntu 22.04LTS

ROS Humble and Iron both use Ubuntu 22.04LTS as their OS.  Unfortunately, the RPi camera v3 needs a later version of the Debian OS and kernel.  This makes things messy!
To install `libcamera` and the necessary tools, please see these [notes](ubuntu/ubuntu.md).

### Raspberry Pi 5, Ubuntu 24.04LTS and cameras

Ubuntu 24.04LTS for the Raspberry Pi does not support the modifications needed to allow cameras to work.  To get around this issue, we installed Raspberry Pi OS so the cameras work and then run ROS in an Ubuntu 24.04LTS docker.  [These notes](ubuntu/docker_ros/docker_ros.md) and related scripts explain how to do this.

## Networking

This section is not Raspberry Pi specific but might help someone to use a robot using a Raspberry Pi in difficult circumstances.

* [Networking in buried pipes.](networking/BuriedPipeNetworkSetup.md)

## Acknowledgments

&copy; 2022-2025, University of Leeds.

The author, A. Blight, has asserted his moral rights.
