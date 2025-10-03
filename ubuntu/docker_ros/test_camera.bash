#!/bin/bash
# Test camera functionality

# Stop on first error
set -e

# Tell the user what is going on
echo
echo "Testing camera functionality..."
echo

# Check if camera tools are available
CAMERA_TOOL=""
if command -v rpicam-hello &> /dev/null; then
    CAMERA_TOOL="rpicam-hello"
elif command -v libcamera-hello &> /dev/null; then
    CAMERA_TOOL="libcamera-hello"
else
    echo "Warning: No camera tools found. Install libcamera first using:"
    echo "  ./ubuntu/pi_camera/install_libcamera.bash"
    echo "  or"
    echo "  ./ubuntu/pi_camera/install_arducam.bash"
    echo
    exit 1
fi

echo "Using camera tool: $CAMERA_TOOL"

# Test camera capture (5 second preview)
echo "Testing camera with 5-second preview..."
if $CAMERA_TOOL --timeout 5000; then
    echo
    echo "Camera test successful!"
else
    echo
    echo "Camera test failed. Please check:"
    echo "1. Camera is properly connected"
    echo "2. Camera is enabled in system configuration"
    echo "3. User is in 'video' group: groups \$USER"
    exit 1
fi

echo
echo "Camera testing completed!"
echo
