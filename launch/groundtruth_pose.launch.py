from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cbf-ros2',
            executable='pose_bridge',
            name='pose_bridge',
            output='screen',
            arguments=['6']
        ),
    ])
