#!/bin/bash
# Install Docker on Raspberry Pi Ubuntu

# Stop on first error
set -e

# Tell the user what is going on
echo
echo "Installing Docker..."
echo

# Check if Docker is already installed
if command -v docker &> /dev/null; then
    echo "Docker is already installed. Skipping installation."
    docker --version
else
    # Update package index
    sudo apt update

    # Install packages to allow apt to use a repository over HTTPS
    sudo apt install -y \
        ca-certificates \
        curl \
        gnupg \
        lsb-release

    # Add Docker's official GPG key
    sudo mkdir -p /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

    # Set up the repository
    echo \
      "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
      $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

    # Update package index again
    sudo apt update

    # Install Docker Engine
    sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

    echo "Docker installation completed successfully."
fi

# Add current user to docker group (requires logout/login to take effect)
echo "Adding user $USER to docker group..."
sudo usermod -aG docker $USER

echo
echo "Docker installation completed. You must log out and log back in for group changes to take effect."
echo