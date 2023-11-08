#!/bin/zsh

# Função para lidar com a interrupção (Ctrl+C)
function handle_ctrl_c() {
    echo "Script interrompido. Salvando o mapa, limpando e encerrando..."

    ros2 run nav2_map_server map_saver_cli -f ./map

    # Adicione comandos de limpeza ou qualquer ação que você deseja realizar antes de encerrar
    exit 1
}

# Configurando o trap para capturar o sinal SIGINT (Ctrl+C)
trap 'handle_ctrl_c &' SIGINT

source ./install/setup.zsh

# Construindo o pacote com colcon
colcon build --packages-select mapping_launch

# Lançando o nó
ros2 launch mapping_launch _launch.xml
