#!/bin/bash
# Raspberry Pi setup script - Part 1
# This script performs installation and configuration steps that require logout/login

# Stop on first error
set -e

echo
echo "=========================================="
echo "Raspberry Pi Setup - Part 1"
echo "=========================================="
echo "This script will install Docker and configure user groups."
echo "After completion, you must log out and log back in before"
echo "running setup_part2.bash to complete the setup."
echo

# Update system first
echo "Updating system packages..."
sudo apt update && sudo apt upgrade -y

echo
echo "Installing Docker and configuring user permissions..."
echo

# Install Docker using modular script
if [ -f "./install_docker.bash" ]; then
    echo "Running Docker installation script..."
    chmod +x ./install_docker.bash
    ./install_docker.bash
else
    echo "Error: install_docker.bash not found!"
    echo "Make sure all setup scripts are in the current directory."
    exit 1
fi

# Add user to video group (for camera access)
echo
echo "Adding user $USER to video group for camera access..."
sudo usermod -aG video $USER

echo
echo "============================================"
echo "PART 1 SETUP COMPLETED SUCCESSFULLY!"
echo "============================================"
echo
echo "IMPORTANT: You must now LOG OUT and LOG BACK IN"
echo "for the group changes to take effect."
echo
echo "After logging back in, complete the setup by running:"
echo "  ./setup_part2.bash"
echo
echo "This will:"
echo "- Build the development container"
echo "- Set up X11 forwarding for GUI applications"
echo "- Test camera functionality"
echo "- Create development directories and example files"
echo
echo "Press Enter to acknowledge and prepare to logout..."
read
echo "Please logout and login, then run: ./setup_part2.bash"
echo