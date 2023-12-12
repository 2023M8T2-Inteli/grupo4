from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_dir + '/launch/turtlebot3_world.launch.py')
    )
    
    cartographer_dir = get_package_share_directory('turtlebot3_cartographer')
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cartographer_dir + '/launch/cartographer.launch.py')
    )

    return LaunchDescription([
        gazebo_launch,
        Node(
            package='vallet',
            executable='teleop_keyboard',
            name='teleop',
            prefix='xterm -e',
            output='screen'
        ),
        cartographer_launch
    ])