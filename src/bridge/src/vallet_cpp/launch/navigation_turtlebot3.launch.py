import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():
    map_file_path = os.path.join(os.getcwd(), 'assets', 'map_turtlebot3.yaml')

    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'param',
            param_file_name))

    nav2_dir = get_package_share_directory('nav2_bringup')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_dir + '/launch/bringup_launch.py'),
        launch_arguments={
            "map": f'{map_file_path}',
            "use_sim_time": 'false',
            "launch-prefix": "xterm -e",
            'params_file': param_dir,
        }.items()
    )

    return LaunchDescription([
        nav2,
        Node(
            package='vallet_cpp',
            executable='logger',
            output='screen',
            name="logger"
        ),
        # Node(
        #     package='vallet',
        #     executable='battery',
        #     output='screen',
        #     name="battery"
        # ),
        Node(
            package='vallet_cpp',
            executable='pose_queue',
            output='screen',
            name="pose_queue",
        ),
        Node(
            package='vallet_cpp',
            executable='vallet',
            output='screen',
            name="vallet",
        ),
        # Node(
        #     package='vallet',
        #     executable='interface',
        #     output='screen',
        #     prefix="xterm -e",
        #     name="interface"
        # )
    ])
