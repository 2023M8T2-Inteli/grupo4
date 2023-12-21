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
            # "launch-prefix": "xterm -e",
            "log_level": "INFO"
        }.items()
    )

    return LaunchDescription([
        rviz,
        Node(
            package='vallet',
            executable='battery',
            output='screen',
            name="battery"
        ),
        Node(
            package='vallet',
            executable='initial_pose',
            output='screen',
            name="initial_pose"
        ),
        Node(
            package='vallet',
            executable='vallet',
            output='screen',
            name="vallet",
        ),
        Node(
            package='vallet',
            executable='task_queue',
            output='screen',
            name="task_queue",
        )
    ])
