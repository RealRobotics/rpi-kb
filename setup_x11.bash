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

# Create X11 helper script for Docker
cat << 'EOF' > run_gui_container.bash
#!/bin/bash
# Script to run Docker container with GUI support

# Allow X11 connections from localhost
xhost +local:root

# Run container with X11 forwarding
docker run -it --rm \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/root/.Xauthority:rw \
    --net=host \
    rpi-robotics:latest

# Revoke X11 access
xhost -local:root
EOF

chmod +x run_gui_container.bash

echo
echo "X11 setup completed!"
echo "Use the 'run_gui_container.bash' script to run containers with GUI support."
echo "Test with: ./run_gui_container.bash"
echo