import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigator',
            executable='chatbot',
            output='screen',
            emulate_tty=True,
            prefix="xterm -e",
            name="chatbot"
        ),
        Node(
            package='navigator',
            executable='init_pose',
            output='screen',
            name="init_pose"
        ),
        Node(
            package='navigator',
            executable='vallet',
            output='screen',
            name="vallet",
        ),
        Node(
            package='navigator',
            executable='queue',
            output='screen',
            name="queue",
            ),
        Node(
            package='navigator',
            executable='coordinatesChatbot',
            output='screen',
            name="coordinatesChatbot",
            emulate_tty=True,
            prefix="xterm -e",
            
            ),
            
    ])