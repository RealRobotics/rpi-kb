#!/bin/bash
# Install and setup Raspberry Pi Camera packages.

# Stop on first error.
set -e

# Tell the user what is going on.
echo
echo "Running $0..."
echo

echo "Installing build tools..."
# From https://askubuntu.com/questions/1529421/camera-setting-up-on-ubuntu-24-for-raspberry-pi-5
sudo apt update
sudo apt install -y --no-install-recommends \
    build-essential \
    libboost-dev \
    libgnutls28-dev openssl libtiff-dev pybind11-dev \
    qtbase5-dev libqt5core5a libqt5widgets5t64 \
    meson cmake \
    python3-yaml python3-ply

# Get libcamera code.
if [ ! -e ~/git/libcamera ]
then
    # Get the code.
    cd ~/git
    git clone https://git.libcamera.org/libcamera/libcamera.git
fi

# libcamera
if [ ! -e /usr/local/bin/cam ]
then
    # Build and install.
    cd ~/git/libcamera
    meson setup build --buildtype=release -Dpipelines=rpi/vc4 \
        -Dipas=rpi/vc4 -Dv4l2=enabled -Dgstreamer=disabled \
        -Dtest=disabled -Dlc-compliance=disabled -Dcam=enabled \
        -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
    ninja -C build
    sudo ninja -C build install
fi

echo
echo "Test using 'cam' ."
echo
echo "$0 took $SECONDS seconds."
echo
