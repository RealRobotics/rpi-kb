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
    deb http://ports.ubuntu.com/ubuntu-ports lunar universe
    deb http://ports.ubuntu.com/ubuntu-ports lunar-updates universe
    deb http://ports.ubuntu.com/ubuntu-ports lunar multiverse
    deb http://ports.ubuntu.com/ubuntu-ports lunar-updates multiverse
    deb http://ports.ubuntu.com/ubuntu-ports lunar-backports main restricted universe multiverse
    deb http://ports.ubuntu.com/ubuntu-ports lunar-security main restricted
    deb http://ports.ubuntu.com/ubuntu-ports lunar-security universe
    deb http://ports.ubuntu.com/ubuntu-ports lunar-security multiverse
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

    Finally, comment out the newly added lines in the file `/etc/apt/sources.list` and then do `sudo apt update` to refresh the index.  This allows us to update and install code from 22.04LTS but still use a much later kernel.

10. Clone this repo and install the `libcamera` and related code:

    ```bash
    cd ~/git
    git clone https://github.com/RealRobotics/rpi-kb.git
    cd rpi-kb/ubuntu/pi_camera
    ./install_arducam.bash
    ```

11. Now we need to tell the system to use the correct overlay for the camera.  Open the config  file using

    ```text
    sudo nano /boot/firmware/config.txt
    ```

    Find the line:

    ```text
    camera_auto_detect=1
    ```

    and change it to:

    ```text
    camera_auto_detect=0
    dtoverlay=imx708
    ```

    Save and reboot.

12. Finally, test the camera using one fo the following options:

    ```bash
    # Display list of cameras.
    cam --list
    # Show 5 seconds of video on screen
    rpicam-hello
    ```

   If these two work, then you have successfully set up your Raspberry Pi.  Time for a nice cup of tea!

## References

* <https://www.raspberrypi.com/documentation/computers/camera_software.html>
* <https://forums.raspberrypi.com/viewtopic.php?t=347172>
* <https://docs.arducam.com/Raspberry-Pi-Camera/Native-camera/12MP-IMX708/>
* <https://docs.arducam.com/Raspberry-Pi-Camera/Native-camera/Libcamera-User-Guide/>
* <https://github.com/raspberrypi/rpicam-apps/issues/551>
* <https://askubuntu.com/questions/1483486/can-i-run-a-kernel-newer-than-5-15-on-ubuntu-22-04-on-a-raspberry-pi-4b>
*
