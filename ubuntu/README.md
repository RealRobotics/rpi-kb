# Ubuntu Installation

Ubuntu is the OS of choice when using ROS and ROS2.  The Raspberry Pi can be used for both 64 and 32 bits installations.  However, this document only deals with installation of the Ubuntu 64 bit OS as the ROS2 packages are pre-complied and can be installed using `apt`.

ROS Humble tested on these releases:

* Ubuntu Desktop 22.04.1 LTS 64 bit.
* Ubuntu Server 22.04.1 LTS 64 bit.

## Installation

The Raspberry Pi imager tool is the best choice for installing images onto an SD card.  Just make sure you chose the correct version of the OS.

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

### Install ROS2 desktop

First, install some useful scripts that make working with git much easier.  Open a terminal and enter:

```bash

```

Now clone the `tankbot` repo and then run the script:


### Raspberry Pi Camera

The setup of the Raspberry Pi Camera Module for Ubuntu 22.04LTS is a bit of a pain as the transition to using `libcamera` had not been fully implemented in this Ubuntu release.  I spent many, many hours trying different ways to make this work and in the end it was quite simple once you realise that you have to build and install the Raspberry Pi specific `libcamera` code.  To build and install the Raspberry Pi `libcamera` libraries and apps, run [this script](install_libcamera.bash).

To run the camera with ROS 2, I found this repo that works well enough for what I needed, so I forked it as I needed to add launch files and a README. <https://github.com/pipebots/camera_ros/tree/add-launch-files>.
