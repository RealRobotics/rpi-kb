# Configuration for ArduCam

The goal is to use the ArduCam B0310 12MP IMX708 camera with ROS Humble on a Raspberry Pi 4.  These instructions also work for the Raspberry Pi camera v3 as it uses the same kernel module and drivers.

The key problems are:

1. The kernel module needed by the ArduCam camera is only available on Linux kernels 6.1 and later.
2. ROS2 Humble (and Iron) only run on Ubuntu 22.04LTS and that uses the Linux kernel 5.15.
3. Ubuntu 22.04LTS does nto have the latest `libcamera` tools or the Raspberry Pi camera utilities `rpicam-hello` etc.

## Process

1. Start with new image on micro-SD card.  Use the Ubuntu 220.04LTS 64 bit desktop image.
2. Power on RPi and set up with English language, UK Keyboard and London time zone.  User name and password as you see fit.
3. Once the UI boots, say no to all the stuff Canonical want you to use.
4. Connect to Eduroam (we need to do lots of updates in a bit).
5. Install git by opening a terminal window and entering:

   ```bash
   sudo apt update
   sudo apt install -y git gitk
   ```

6. Install Andy's magic scripts to make using Git easier.

   ```bash
   mkdir ~/git
   cd ~/git
   git clone https://github.com/andyblight/bash_scripts.git
   cd bash_scripts
   ./install.sh ubuntu22.04lts
   ```

7. Disable automatic updates as follows:
    Edit `/etc/apt/apt.conf.d/20auto-upgrades` and modify the two lines to look like this

    ```text
    APT::Periodic::Update-Package-Lists "0";
    APT::Periodic::Unattended-Upgrade "0"
    ```

8. Do a full upgrade.

   ```bash
   upgrade.sh
   ```

    You'll probably need to reboot after this upgrade.

9. Install the 6.2 kernel.  Edit the APT sources file `sudo nano /etc/apt/sources.list` and add these lines to the end:

    ```text
    # adding this to get the new 6.2.x kernel from lunar
    deb http://ports.ubuntu.com/ubuntu-ports lunar main restricted
    deb http://ports.ubuntu.com/ubuntu-ports lunar-updates main restricted
    ```

    Then execute the following commands:

    ```bash
    sudo apt update
    sudo apt install linux-image-6.2.0-1017-raspi linux-raspi-headers-6.2.0-1017 linux-modules-6.2.0-1017-raspi linux-raspi-tools-6.2.0-1017
    ```

    And reboot again.  Once booted, verify that the new kernel is installed as follows:

    ```text
    $ uname -a
    Linux ball-desktop 6.2.0-1017-raspi #19-Ubuntu SMP PREEMPT Mon Nov 13 15:35:19 UTC 2023 aarch64 aarch64 aarch64 GNU/Linux
    ```

10. Clone this repo and install the `libcamera` and related code:

   ```bash
   cd ~/git
   git clone https://github.com/RealRobotics/rpi-kb.git
   cd rpi-kb/ubuntu/pi_camera
   ./install_arducam.bash
   ```

## ArduCam notes

From <https://docs.arducam.com/Raspberry-Pi-Camera/Native-camera/12MP-IMX708/>

Notes

Please make sure you are running the latest version of Raspberry Pi OS. (January 28th，2022 or later releases, Debian version:11(Bullseye)). You need to update the config file and use libcamera apps.
The official IMX708 Camera Module 3 can be used on Raspberry Pi directly(B0306, B0307). Rest of the IMX708 Camera Module will need some modification on configuration, please refer to the following content:

For Raspberry Bullseye users running on Pi 4, please do the following:

```bash
sudo nano /boot/config.txt
#Find the line: camera_auto_detect=1, update it to:
camera_auto_detect=0
dtoverlay=imx708
#Save and reboot.
```

For Bullseye users running on Pi 0 ~ 3, please also:

```text
Open a terminal
Run sudo raspi-config
Navigate to Advanced Options
Enable Glamor graphic acceleration
Reboot your Pi
```

If you encounter the display issues, please also execute the following steps:

```text
Open a terminal
Run sudo raspi-config
Navigate to Advanced Options
Navigate to GL Driver
Select GL (Full KMS)
Reboot your Pi
```

## References

* <https://www.raspberrypi.com/documentation/computers/camera_software.html>
* <https://forums.raspberrypi.com/viewtopic.php?t=347172>
* <https://docs.arducam.com/Raspberry-Pi-Camera/Native-camera/12MP-IMX708/>
* <https://docs.arducam.com/Raspberry-Pi-Camera/Native-camera/Libcamera-User-Guide/>
* <https://github.com/raspberrypi/rpicam-apps/issues/551>
* <https://askubuntu.com/questions/1483486/can-i-run-a-kernel-newer-than-5-15-on-ubuntu-22-04-on-a-raspberry-pi-4b>
*
