#!/bin/bash
# Install Docker and dependencies for Raspberry Pi 5

# Stop on first error.
set -e

# Tell the user what is going on.
echo
echo "Running $0..."
echo "Installing Docker Engine and dependencies for Raspberry Pi 5"
echo

# Check if running on Raspberry Pi
if ! grep -q "Raspberry Pi" /proc/device-tree/model 2>/dev/null; then
    echo "Warning: This script is designed for Raspberry Pi. Continuing anyway..."
fi

# Update package list
echo "Updating package list..."
sudo apt-get update

# Install prerequisites
echo "Installing prerequisites..."
sudo apt-get install -y \
    ca-certificates \
    curl \
    gnupg \
    lsb-release

# Create directory for Docker's GPG key
sudo install -m 0755 -d /etc/apt/keyrings

# Download and install Docker's official GPG key
echo "Adding Docker's official GPG key..."
sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add Docker repository
echo "Adding Docker repository..."
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Update package list with Docker repository
echo "Updating package list with Docker repository..."
sudo apt-get update

# Install Docker Engine, CLI, containerd, and plugins
echo "Installing Docker Engine..."
sudo apt-get install -y \
    docker-ce \
    docker-ce-cli \
    containerd.io \
    docker-buildx-plugin \
    docker-compose-plugin

# Add current user to docker group
echo "Adding user $USER to docker group..."
sudo usermod -aG docker $USER

# Enable Docker service
echo "Enabling Docker service..."
sudo systemctl enable docker
sudo systemctl start docker

# Configure camera access permissions
echo "Configuring camera access permissions..."

# Add user to video group
sudo usermod -aG video $USER

# Create udev rules for camera access
echo 'SUBSYSTEM=="vchiq", GROUP="video", MODE="0660"' | sudo tee /etc/udev/rules.d/10-vchiq-permissions.rules
echo 'SUBSYSTEM=="dma_heap", GROUP="video", MODE="0660"' | sudo tee /etc/udev/rules.d/20-dma-heap.rules

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Enable camera interface if not already enabled
if command -v raspi-config >/dev/null 2>&1; then
    echo "Enabling camera interface..."
    sudo raspi-config nonint do_camera 0
else
    echo "raspi-config not found, skipping camera enable step"
fi

# Test Docker installation
echo "Testing Docker installation..."
if sudo docker run --rm hello-world > /dev/null 2>&1; then
    echo "✓ Docker installation successful!"
else
    echo "✗ Docker installation may have failed. Please check manually."
fi

echo
echo "Docker installation complete!"
echo
echo "IMPORTANT: You need to log out and log back in (or reboot) for the"
echo "docker group membership to take effect."
echo
echo "After logging back in, you can test Docker with:"
echo "  docker run --rm hello-world"
echo
echo "To verify camera access, check:"
echo "  groups \$USER  # Should include 'docker' and 'video'"
echo "  ls -la /dev/video*  # Should show camera devices"
echo
echo "$0 took $SECONDS seconds."
echo