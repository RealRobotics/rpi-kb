# Ubuntu Installation

Ubuntu is the OS of choice when using ROS and ROS2.  The Raspberry Pi can be used for both 64 and 32 bits installations.  However, this document only deals with installation of the Ubuntu 64 bit OS as the ROS2 packages are pre-complied and can be installed using `apt`.

ROS Humble tested on these releases:

* Ubuntu Desktop 22.04.1 LTS 64 bit.
* Ubuntu Server 22.04.1 LTS 64 bit.

ROS Jazzy tested on this release:

* Ubuntu Desktop 24.04.3 LTS 64 bit.

## Installation

The Raspberry Pi imager tool is the best choice for installing images onto an SD card.  Just make sure you chose the correct version of the OS.

Once the image has been created, install the SD card in the RPi and then go through the configuration screens.   Select:

* English
* UK keyboard
* London as the time zone.
* Set user name and password.  Use something you won't forget!
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

## Libcamera

Raspberry Pis use a custom version of the libcamera library that is not provided with Ubuntu, even as a new module, so we have to build it ourselves.  There are a few different ways I have done this, and they are:

* [Ubuntu 24.04LTS on a Pi5 with Pi Camera Module 3](ubuntu/pi_camera/pi5/libcamera.md)
* [Ubuntu 22.04LTS on a Pi4 with Pi Camera Module 2](ubuntu/pi_camera/pi5/libcamera.md)
* [Ubuntu 22.04LTS on a Pi4 with Arducam](ubuntu/pi_camera/pi5/libcamera.md)

### Why doesn't Ubuntu ship with the Raspberry Pi version of libcamera?

A good question.  My guess is that there could have been a conversation that went like this.

"So we need to build Ubuntu for the Raspberry Pi. That's an ARM chip, so let's build that."
"Oops!.  It doesn't work, so lets patch the kernel."
"Great, it builds.  Ship it!"

Then all the forums complain first time they try to use a camera, and no-one has time to fix the problem.
