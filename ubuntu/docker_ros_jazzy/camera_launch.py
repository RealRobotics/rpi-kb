"""
Launch file for Raspberry Pi camera node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for camera node."""
    
    # Declare launch arguments
    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='0',
        description='Camera device index'
    )
    
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='640',
        description='Camera frame width'
    )
    
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='480',
        description='Camera frame height'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30.0',
        description='Camera frames per second'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_frame',
        description='Camera frame ID'
    )
    
    # Camera publisher node
    camera_node = Node(
        package='camera_package',  # This would be the actual package name
        executable='camera_publisher.py',
        name='camera_publisher',
        parameters=[{
            'camera_index': LaunchConfiguration('camera_index'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'fps': LaunchConfiguration('fps'),
            'frame_id': LaunchConfiguration('frame_id'),
        }],
        output='screen'
    )
    
    # Alternative using image_tools package (if available)
    image_tools_node = Node(
        package='image_tools',
        executable='cam2image',
        name='camera_image_tools',
        parameters=[{
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'frequency': LaunchConfiguration('fps'),
        }],
        # Uncomment to use this instead of custom camera_publisher
        # condition=IfCondition('false')
    )
    
    return LaunchDescription([
        # Launch arguments
        camera_index_arg,
        width_arg,
        height_arg,
        fps_arg,
        frame_id_arg,
        
        # Nodes
        # camera_node,  # Custom camera publisher
        image_tools_node,  # Using standard image_tools
    ])