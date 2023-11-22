# Real Robotics Raspberry Pi knowledge base

This repo contains information on how to set up Raspberry Pis for work with robotics and measurement platforms at the University of Leeds.

## SD card images

Raspberry Pi provides a nice imager tool to download and create an SD card image for the operating system (OS) along with great instructions, so use them! [See here for details.](https://www.raspberrypi.com/software/)

For robotics use, the preferred OS is Ubuntu 20.04 or 22.04, 64 bit.  This is best if you use ROS as the packages are prebuilt for the Pi so can be installed without building from source.

For general purpose use, the Raspberry Pi OS with desktop (32 bit) is best. This release has all the recommended software and has the best online support.

The micro SD card should be as fast as possible and 16GB or larger.  My personal preference is the 32GB SanDisk Extreme micro SD cards.  You can get them from [Amazon](https://www.amazon.co.uk/SanDisk-Extreme-microSDHC-Adapter-Performance/dp/B06XWMQ81P/ref=asc_df_B06XWMQ81P/) or Science Warehouse.

### Back up and restore an image

Once you have prepared an SD card, it is useful to be able to create a back up copy and restore the copy when needed.  This process is documented [here](backup/backup.md).

### Customising Ubuntu 22.04LTS for the Raspberry Pi

There are some useful tricks and tips for setting up Ubuntu for robotics use [here](ubuntu/README.md).  There are also some notes on using `libcamera` on the Raspberry Pi (22.04LTS does not have this working so you have to do a fair bit manually).

## Eduroam

At home, it is natural to connect a Raspberry Pi to the internet using your home Wi-Fi.  Eduroam is the Wi-Fi network for the university so it is natural for Raspberry Pis to be connected to Eduroam at some point.

__However, due to the often lax security on Raspberry Pis caused by poor passwords etc., it is recommended that the Raspberry Pis are only connected to Eduroam during updates etc. and are disconnected afterwards.__

The documentation for connecting to Eduroam is [here](eduroam/eduroam.md).

## Acknowledgments

&copy; 2022, University of Leeds.

The author, A. Blight, has asserted their moral rights.
