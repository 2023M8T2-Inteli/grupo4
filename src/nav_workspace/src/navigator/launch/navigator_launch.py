import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    
    gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    gazebo = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                    gazebo_dir + '/launch/turtlebot3_world.launch.py'))
    
    rviz_dir = get_package_share_directory('turtlebot3_navigation2')
    print(rviz_dir)
    rviz = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(rviz_dir + '/launch/navigation2.launch.py'),
                launch_arguments={
                   "map": "my-map.yaml",
                   "use_sim_time": 'true',
                   "log_level": "INFO"
                }.items()
            )
    

    return LaunchDescription([
        rviz,
        gazebo,
        Node(
            package='navigator',
            executable='cli',
            output='screen',
            emulate_tty=True,
            prefix="xterm -e",
            name="cli"
        ),
        Node(
            package='navigator',
            executable='init_pose',
            output='screen',
            name="init_pose"
        ),
        Node(
            package='navigator',
            executable='andando',
            output='screen',
            name="andando",
            prefix="xterm -e",
        ),
        Node(
            package='navigator',
            executable='queue',
            output='screen',
            name="queue",
            prefix="xterm -e",
            ),
            
    ])