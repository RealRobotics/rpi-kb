#!/bin/bash

# Create image file on local hard drive.
# Check that mmcblk0 is used for the SD card.
sudo dd if=/dev/mmcblk0 of=/tmp/myimg.img bs=16M status=progress

# Shirnk the image.
sudo pishrink.sh -vrza /tmp/myimg.img
