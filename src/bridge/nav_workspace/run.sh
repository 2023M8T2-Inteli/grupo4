#!/bin/bash

# Default map path
DEFAULT_MAP_PATH="./maps/my-map.yaml"

# Check if a map path is provided
if [ -z "$1" ]; then
    echo "No map path provided. Using the default map path: $DEFAULT_MAP_PATH"
    MAP_PATH=$DEFAULT_MAP_PATH
else
    MAP_PATH=$1
fi


    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:="$MAP_PATH" &
    ros2 launch ./src/navigator/launch/navigator_launch.py &


# Aguarda o processo ROS terminar
wait $ros_pid
