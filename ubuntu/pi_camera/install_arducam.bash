#!/bin/bash
# Install and setup Raspberry Pi Camera packages.

# Stop on first error.
set -e

# Tell the user what is going on.
echo
echo "Running $0..."
echo

# Add build dependencies.  Should be pretty much the minimal set required.
sudo apt update
sudo apt install -y --no-install-recommends \
    g++ meson ninja-build pkg-config \
    libyaml-dev python3-yaml python3-ply python3-jinja2 \
    libssl-dev openssl \
    libdw-dev libunwind-dev \
    libudev-dev libboost-dev \
    libglib2.0-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libevent-dev \
    libcamera-dev libjpeg-dev libtiff5-dev \
    libavcodec-dev libavdevice-dev libavformat-dev libswresample-dev \
    libgnutls28-dev openssl libtiff5-dev \
    qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5 \
    python3-pip
sudo pip3 install pyyaml ply
sudo pip3 install --upgrade meson

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
