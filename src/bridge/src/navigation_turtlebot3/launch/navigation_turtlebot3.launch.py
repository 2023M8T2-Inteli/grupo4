import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    rviz_dir = get_package_share_directory('turtlebot3_navigation2')
    map_file_path = os.path.join(os.getcwd(), 'assets', 'map_turtlebot3.yaml')
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            rviz_dir + '/launch/navigation2.launch.py'),
        launch_arguments={
            "map": f'{map_file_path}',
            "use_sim_time": 'false',
                   "log_level": "INFO"
        }.items()
    )

    return LaunchDescription([
        rviz,
        Node(
            package='navigation_turtlebot3',
            executable='initial_pose',
            output='screen',
            name="initial_pose"
        ),
        Node(
            package='navigation_turtlebot3',
            executable='vallet',
            output='screen',
            name="vallet",
        ),
        Node(
            package='navigation_turtlebot3',
            executable='pose_queue',
            output='screen',
            name="pose_queue",
        ),
        Node(
            package='navigation_turtlebot3',
            executable='interface',
            output='screen',
            prefix="xterm -e",
            name="interface"
        ),
        Node(
            package='navigation_turtlebot3',
            executable='chatbot',
            output='screen',
            prefix="xterm -e",
            name="chatbot"
        )
    ])
