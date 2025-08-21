#!/bin/bash
# Test camera functionality inside the Docker container

# Stop on first error.
set -e

echo
echo "Running camera tests..."
echo

# Test 1: List available video devices
echo "=== Test 1: Video devices ==="
if command -v v4l2-ctl >/dev/null 2>&1; then
    echo "Available video devices:"
    v4l2-ctl --list-devices || echo "No video devices found or v4l2-ctl failed"
else
    echo "v4l2-ctl not available"
fi

echo
ls -la /dev/video* 2>/dev/null || echo "No /dev/video* devices found"

# Test 2: Check camera with libcamera
echo
echo "=== Test 2: libcamera test ==="
if command -v libcamera-hello >/dev/null 2>&1; then
    echo "Testing camera with libcamera-hello (5 second test)..."
    timeout 10s libcamera-hello --timeout 5000 --nopreview || echo "libcamera-hello test failed or timed out"
else
    echo "libcamera-hello not available"
fi

# Test 3: Check ROS 2 camera tools
echo
echo "=== Test 3: ROS 2 camera tools ==="
if command -v ros2 >/dev/null 2>&1; then
    echo "ROS 2 is available"
    echo "Available ROS 2 packages:"
    ros2 pkg list | grep -E "(camera|image)" | head -10
    
    echo
    echo "Testing image_tools cam2image (will run for 5 seconds)..."
    timeout 10s ros2 run image_tools cam2image --ros-args -p width:=640 -p height:=480 || echo "cam2image test failed or timed out"
else
    echo "ROS 2 not available"
fi

# Test 4: Check OpenCV
echo
echo "=== Test 4: OpenCV test ==="
python3 -c "
import cv2
print(f'OpenCV version: {cv2.__version__}')
try:
    cap = cv2.VideoCapture(0)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            print(f'Camera capture successful: frame shape {frame.shape}')
        else:
            print('Camera capture failed: could not read frame')
        cap.release()
    else:
        print('Camera capture failed: could not open camera')
except Exception as e:
    print(f'OpenCV test failed: {e}')
"

echo
echo "Camera tests completed."
echo