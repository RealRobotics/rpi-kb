# Configuration for ArduCam

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
