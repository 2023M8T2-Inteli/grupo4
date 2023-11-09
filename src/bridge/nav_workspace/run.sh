#!/bin/bash

# Default map path
DEFAULT_MAP_PATH="./maps/map.yaml"

# Check if a map path is provided
if [ -z "$1" ]; then
    echo "No map path provided. Using the default map path: $DEFAULT_MAP_PATH"
    MAP_PATH=$DEFAULT_MAP_PATH
else
    MAP_PATH=$1
fi

# Função para lançar o processo ROS e capturar o PID
launch_ros_process() {
    # Run the first command with the provided or default map path in the background
    ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:="$MAP_PATH" &

# Run the second command in the background
    ros2 launch ./src/navigator/launch/navigator_launch.py &
  local use_sim_time_value=${1:-False}
  ros_pid=$!
  echo "ROS launch started with PID: $ros_pid"
}



# Função chamada quando o CTRL+C é pressionado
handle_ctrl_c() {

    # Encerra o processo ROS se ainda estiver em execução
    if kill -0 $ros_pid > /dev/null 2>&1; then
        kill_ros_process
    fi

    exit 0
}

# Define o manipulador para SIGINT (CTRL+C)

# Lança o processo ROS
launch_ros_process

# Aguarda o processo ROS terminar
wait $ros_pid
