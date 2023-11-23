from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='streamer',
            executable='streamer',
            output='screen',
            name='streamer'
        )
    ])