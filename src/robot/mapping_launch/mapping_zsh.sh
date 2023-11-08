#!/bin/zsh
source ./install/setup.zsh
colcon build --packages-select mapping_launch
ros2 launch mapping_launch _launch.xml