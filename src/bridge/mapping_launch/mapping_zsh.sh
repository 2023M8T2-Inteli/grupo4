#!/bin/zsh

# Inicializa o ambiente ROS 2
source /opt/ros/humble/setup.zsh

# Compila o pacote desejado
colcon build --packages-select mapping_launch

# Inicializa o ambiente ROS 2
source ./install/setup.zsh

# Função para lançar o processo ROS e capturar o PID
launch_ros_process() {
  local use_sim_time_value=${1:-false}
  ros2 launch mapping_launch _launch.xml use_sim_time:=$use_sim_time_value &
  ros_pid=$!
  echo "ROS launch started with PID: $ros_pid"
}

# Função para encerrar o processo ROS
kill_ros_process() {
  echo "Encerrando o processo ROS com PID: $ros_pid"
  kill -9 $ros_pid
}

# Função chamada quando o CTRL+C é pressionado
handle_ctrl_c() {
    echo "CTRL+C pressionado. Salvando o mapa..."
    # Salva o mapa. Não precisa executar em background ou capturar o PID,
    # já que é a última ação antes do script terminar.
    ros2 run nav2_map_server map_saver_cli -f ./assets/map/map

    # Encerra o processo ROS se ainda estiver em execução
    if kill -0 $ros_pid > /dev/null 2>&1; then
        kill_ros_process
    fi

    exit 0
}

# Define o manipulador para SIGINT (CTRL+C)
trap handle_ctrl_c SIGINT

# Lança o processo ROS
launch_ros_process

# Aguarda o processo ROS terminar
wait $ros_pid
