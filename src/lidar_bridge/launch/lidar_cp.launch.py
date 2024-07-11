from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_bridge',  # Replace with your package name
            executable='lidar_transform',  # Replace with your node executable name
            name='lidar_transform',
            #output='screen',
        ),
    ])

