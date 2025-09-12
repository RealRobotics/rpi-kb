#!/bin/bash
# Helper script for common ROS Jazzy operations

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to display help
show_help() {
    echo "ROS Jazzy Helper Script"
    echo ""
    echo "Usage: $0 [COMMAND]"
    echo ""
    echo "Commands:"
    echo "  build       - Build the workspace"
    echo "  source      - Source ROS Jazzy and workspace environments"
    echo "  create PKG  - Create a new ROS package"
    echo "  test        - Run tests"
    echo "  clean       - Clean build artifacts"
    echo "  demo        - Run demo nodes"
    echo "  help        - Show this help message"
}

# Source ROS environment
source_ros() {
    echo -e "${GREEN}Sourcing ROS Jazzy environment...${NC}"
    source /opt/ros/jazzy/setup.bash
    
    if [ -f install/setup.bash ]; then
        echo -e "${GREEN}Sourcing workspace environment...${NC}"
        source install/setup.bash
    fi
}

# Build workspace
build_workspace() {
    echo -e "${GREEN}Building ROS workspace...${NC}"
    source_ros
    colcon build --symlink-install
    echo -e "${GREEN}Build complete!${NC}"
    echo "Remember to source the workspace: source install/setup.bash"
}

# Create new package
create_package() {
    if [ -z "$1" ]; then
        echo "Usage: $0 create PACKAGE_NAME"
        return 1
    fi
    
    echo -e "${GREEN}Creating ROS package: $1${NC}"
    cd src
    ros2 pkg create --build-type ament_python "$1" --dependencies rclpy
    cd ..
    echo -e "${GREEN}Package $1 created in src/$1${NC}"
}

# Run tests
run_tests() {
    echo -e "${GREEN}Running ROS tests...${NC}"
    source_ros
    colcon test
    colcon test-result --verbose
}

# Clean workspace
clean_workspace() {
    echo -e "${YELLOW}Cleaning workspace...${NC}"
    rm -rf build/ install/ log/
    echo -e "${GREEN}Workspace cleaned!${NC}"
}

# Run demo
run_demo() {
    echo -e "${GREEN}Starting ROS Jazzy demo (talker/listener)...${NC}"
    echo "In one terminal: ros2 run demo_nodes_py talker"
    echo "In another terminal: ros2 run demo_nodes_py listener"
    echo ""
    echo "Press Ctrl+C to stop the demo"
    ros2 run demo_nodes_py talker
}

# Main script logic
case "$1" in
    build)
        build_workspace
        ;;
    source)
        source_ros
        echo "ROS environment sourced!"
        ;;
    create)
        create_package "$2"
        ;;
    test)
        run_tests
        ;;
    clean)
        clean_workspace
        ;;
    demo)
        run_demo
        ;;
    help|--help|-h|"")
        show_help
        ;;
    *)
        echo "Unknown command: $1"
        show_help
        exit 1
        ;;
esac