#!/usr/bin/env python3
"""
Simple ROS 2 camera publisher node for Raspberry Pi camera.
Publishes camera images to /camera/image_raw topic.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraPublisher(Node):
    """ROS 2 node that publishes camera images."""
    
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Declare parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('frame_id', 'camera_frame')
        
        # Get parameters
        camera_index = self.get_parameter('camera_index').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Initialize camera
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera {camera_index}')
            return
            
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        
        # Log actual camera properties
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        self.get_logger().info(f'Camera opened: {actual_width}x{actual_height} @ {actual_fps} FPS')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create publisher
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        
        # Create timer
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Camera publisher node started')
    
    def timer_callback(self):
        """Timer callback to capture and publish camera frames."""
        ret, frame = self.cap.read()
        if ret:
            try:
                # Convert OpenCV image to ROS Image message
                msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                
                # Publish the image
                self.publisher.publish(msg)
                
            except Exception as e:
                self.get_logger().error(f'Failed to publish image: {e}')
        else:
            self.get_logger().warn('Failed to capture frame')
    
    def destroy_node(self):
        """Clean up resources when node is destroyed."""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    node = CameraPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()