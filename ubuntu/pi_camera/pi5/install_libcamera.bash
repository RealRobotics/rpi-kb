#!/bin/bash
#
# Script to build and install Raspberry Pi's libcamera fork and
# the ROS 2 camera_ros package for Ubuntu 24.04 (Jazzy) on a Raspberry Pi 5.
# This is necessary because the standard Ubuntu libcamera does not support
# the custom CSI camera modules on the RPi 5.

# Stop on first error.
set -e

# Tell the user what is going on.
echo
echo "Running $0..."
echo

# --- Configuration ---
# The official RPi forks are required for Pi 5 camera support
LIBCAMERA_REPO="https://github.com/raspberrypi/libcamera.git"
LIBPISP_REPO="https://github.com/raspberrypi/libpisp.git"

echo "Starting build and installation of Raspberry Pi Camera libraries..."

# Remove the ROS default libcamera package (if present) to avoid conflicts
if [ "$ROS_DISTRO" != "" ]
then
    if [ $(dpkg -s ros-$ROS_DISTRO-libcamera &> /dev/null; echo $?) -eq 0 ]
    then
        echo "Removing conflicting ROS libcamera package..."
        sudo apt remove -y ros-$ROS_DISTRO-libcamera
    fi
fi

# Dependencies.  Should be pretty much the minimal set required.
echo "Installing build dependencies..."
sudo apt update
sudo apt install -y --no-install-recommends \
    git cmake meson ninja-build build-essential libevent-dev \
    python3-pip python3-yaml python3-ply libboost-dev libgnutls28-dev \
    openssl libtiff-dev libdrm-dev libexpat1-dev libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev libjpeg-dev libpng-dev \
    libsdl2-dev qt6-base-dev

# libpisp
if [ ! -e /usr/local/lib/libpisp.so ]
then
    # Get the code.
    if [ ! -e ~/git/libpisp ]
    then
        mkdir -p ~/git
        cd ~/git
        git clone $LIBPISP_REPO
    fi
    cd ~/git/libpisp
    # Build and install
    meson setup build --buildtype=release
    ninja -C build
    sudo ninja -C build install
fi

# libcamera
if [ ! -e /usr/local/bin/qcam ]
then
    # Get the code.
    if [ ! -e ~/git/libcamera ]
    then
        mkdir -p ~/git
        cd ~/git
        git clone $LIBCAMERA_REPO
    fi
    cd ~/git/libcamera

    meson setup build --buildtype=release \
        -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp \
        -Dv4l2=enabled -Dgstreamer=enabled -Dlc-compliance=enabled \
        -Dcam=enabled -Dqcam=enabled -Ddocumentation=disabled \
        -Dtest=false

    ninja -C build
    sudo ninja -C build install
    # Update dynamic linker run-time bindings
    sudo ldconfig
fi

# Add user to video group
sudo adduser $USER video

echo
echo "To list available cameras, run: cam -l"
echo "To see the video on screen, run: qcam"
echo
echo "You may need to log out and in again for video group changes to take effect."
echo
echo "$0 took $SECONDS seconds."
echo
