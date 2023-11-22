#!/bin/bash

# Based on this page: https://github.com/raspberrypi/libcamera
# There is a more detailed blog post somewhere but I cant find it!

echo "TO DO!"

sudo apt install -y \
    libyaml-dev python3-yaml python3-ply python3-jinja2 \
    libdw-dev libunwind-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5 qttools5-dev-tools libtiff-dev

pip3 install --user meson
pip3 install --user --upgrade meson

echo "Finish this off"
