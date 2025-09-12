#!/bin/bash
# Setup X11 forwarding for GUI applications in Docker

# Stop on first error
set -e

# Tell the user what is going on
echo
echo "Setting up X11 forwarding for GUI applications..."
echo

# Install X11 dependencies
sudo apt update
sudo apt install -y \
    x11-xserver-utils \
    xauth \
    x11-apps

echo
echo "X11 setup completed!"
echo
