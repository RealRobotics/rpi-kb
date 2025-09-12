#!/bin/bash
# Setup script for ROS Jazzy Docker environment on Raspberry Pi

# Stop on first error
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}Setting up ROS Jazzy Docker environment...${NC}"

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo -e "${RED}Docker is not installed. Please install Docker first.${NC}"
    echo "You can install Docker using:"
    echo "curl -fsSL https://get.docker.com -o get-docker.sh"
    echo "sudo sh get-docker.sh"
    exit 1
fi

# Check if Docker Compose is available
if ! command -v docker-compose &> /dev/null && ! docker compose version &> /dev/null; then
    echo -e "${RED}Docker Compose is not available. Please install Docker Compose.${NC}"
    exit 1
fi

# Create workspace directory if it doesn't exist
if [ ! -d "workspace" ]; then
    mkdir -p workspace
    echo -e "${YELLOW}Created workspace directory${NC}"
fi

# Build the Docker image
echo -e "${GREEN}Building ROS Jazzy Docker image...${NC}"
docker build -t rpi-ros-jazzy:latest .

echo -e "${GREEN}Setup complete!${NC}"
echo ""
echo "To start the ROS Jazzy container, run:"
echo "  ./run_container.sh"
echo ""
echo "Or use Docker Compose:"
echo "  docker-compose up -d"
echo "  docker-compose exec ros_jazzy bash"