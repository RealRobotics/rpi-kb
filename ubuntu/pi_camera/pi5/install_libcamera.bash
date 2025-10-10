#!/bin/bash
# Install and setup Raspberry Pi libcamera packages.

# Stop on first error.
set -e

# Tell the user what is going on.
echo
echo "Running $0..."
echo

# libcamera
if [ ! -e /usr/local/bin/cam ]
then
    # Get the code.
    cd ~/git
    git clone https://git.libcamera.org/libcamera/libcamera.git
    cd libcamera
    # Dependencies.  Should be pretty much the minimal set required.
    sudo apt update
    sudo apt install -y --no-install-recommends \
        python3-pip git python3-jinja2 \
        libboost-dev \
        libgnutls28-dev openssl libtiff5-dev pybind11-dev \
        qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5 \
        meson cmake \
        python3-yaml python3-ply \
        libglib2.0-dev libgstreamer-plugins-base1.0-dev \
        cmake libboost-program-options-dev libdrm-dev libexif-dev \
        meson ninja-build


        # g++ meson cmake ninja-build pkg-config \
        # libyaml-dev python3-yaml python3-ply python3-jinja2 \
        # libssl-dev openssl \
        # libdw-dev libunwind-dev \
        # libudev-dev libboost-dev \
        # libglib2.0-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
        # libevent-dev libexif-dev \
        # libcamera-dev libjpeg-dev libtiff5-dev libpng-dev \
        # libavcodec-dev libavdevice-dev libavformat-dev libswresample-dev \
        # libgnutls28-dev openssl \
        # qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5 \
        # python3-pip

    # Install meson and related python packages.
    sudo pip3 install pyyaml ply
    sudo pip3 install --upgrade meson
    # Build and install.
    meson build --buildtype=release -Dpipelines=raspberrypi \
        -Dipas=raspberrypi -Dv4l2=true -Dgstreamer=enabled \
        -Dtest=false -Dlc-compliance=disabled -Dcam=enabled \
        -Dqcam=enabled -Ddocumentation=disabled
    ninja -C build
    sudo ninja -C build install
fi

# rpicam-apps
if [ ! -e /usr/local/bin/rpicam-hello ]
then
    # Get the code.
    cd ~/git
    git clone https://github.com/raspberrypi/rpicam-apps.git
    cd rpicam-apps/
    # Install dependencies.
    sudo apt update
    sudo apt install -y --no-install-recommends \
        libboost-program-options-dev libdrm-dev libexif-dev \
        meson ninja-build
    # Build the code.
    # NOTE: the last three options disable OpenCV, TensorFlow Lite and Hailo.  NOT TESTED with any enabled.
    meson setup build -Denable_libav=enabled -Denable_drm=enabled -Denable_egl=enabled -Denable_qt=enabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled
    sudo meson install -C build
fi

# Add user to video group
sudo adduser $USER video

echo
echo "Test using 'qcam' or 'libcamera-hello --qt-preview'."
echo
echo "$0 took $SECONDS seconds."
echo
