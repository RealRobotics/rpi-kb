from setuptools import setup, find_packages

package_name = 'rpi_camera_example'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camera_launch.py']),
        ('share/' + package_name + '/config', ['config/camera_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RPI User',
    maintainer_email='user@example.com',
    description='Example ROS Jazzy package for Raspberry Pi camera',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = rpi_camera_example.camera_publisher:main',
        ],
    },
)