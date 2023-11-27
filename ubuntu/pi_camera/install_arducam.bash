#!/bin/bash
# Install and setup Raspberry Pi Camera packages.

# Stop on first error.
set -e

# Tell the user what is going on.
echo
echo "Running $0..."
echo

# libepoxy
if [ ! -e /usr/local/lib/aarch64-linux-gnu/libepoxy.so ]
then
    cd ~/git
    git clone https://github.com/anholt/libepoxy.git
    cd libepoxy
    mkdir _build
    cd _build
    meson
    ninja
    sudo ninja install
    sudo ldconfig
fi

if [ ! -e /usr/local/lib/aarch64-linux-gnu/libpisp.so ]
then
    cd ~/git
    git clone https://github.com/raspberrypi/libpisp.git
    cd libpisp/
    meson setup build
    meson compile -C build
    sudo meson install -C build
    sudo ldconfig
fi

# libcamera
if [ ! -e /usr/local/bin/cam ]
then
    cd ~/git
    git clone https://github.com/raspberrypi/libcamera
    cd libcamera
    meson setup build
    ninja -C build install
    sudo ldconfig
fi

# Raspberry Pi camera apps
if [ ! -e /usr/local/bin/rpicam-hello ]
then
    cd ~/git
    git clone https://github.com/raspberrypi/rpicam-apps.git
    cd rpicam-apps
    git checkout v1.2.2
    meson setup build -Denable_libav=true -Denable_drm=true -Denable_egl=true -Denable_qt=true -Denable_opencv=false -Denable_tflite=false
    meson compile -C build
    sudo meson install -C build
    sudo ldconfig
fi

# Add user to video group
sudo adduser $USER video

echo
echo "Test using 'qcam' or 'rpicam-hello --qt-preview'."
echo
echo "$0 took $SECONDS seconds."
echo
