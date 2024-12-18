#!/bin/bash

# Create image file on local hard drive from the SD card on mmcblk0.
sudo dd if=/dev/mmcblk0 of=./myimg.img bs=16M status=progress

# Shrink the image.
sudo pishrink.sh -vrza ./myimg.img
