# Stage 1: Build Stage
FROM henriquemarlon/ros:humble-vallet-desktop AS builder
# FROM osrf/ros:humble-desktop

WORKDIR /bridge

# Copia o código da aplicação para o diretório de trabalho
COPY ./src .

# Builda o pacote
RUN /bin/bash -c ' \
   sudo apt-get update -y && \
   sudo apt-get install ros-dev-tools -y && \
   source /opt/ros/humble/setup.bash && \
   colcon build \
   '

# Stage 2: Runtime Stage
FROM henriquemarlon/ros:humble-vallet-desktop

WORKDIR /bridge

# Copia apenas os artefatos necessários do Stage 1
COPY --from=builder /bridge/install /bridge/install
COPY --from=builder /bridge/build /bridge/build

# Criar o script entrypoint.sh
RUN echo "#!/bin/bash" > /entrypoint.sh && \
   echo "source /opt/ros/humble/setup.bash" >> /entrypoint.sh && \
   echo "source /bridge/install/setup.bash" >> /entrypoint.sh && \
   echo "export TURTLEBOT3_MODEL=burger" >> /entrypoint.sh && \
   echo "export ROS_DOMAIN_ID=50" >> /entrypoint.sh && \
   echo "export SOCKET_SERVER_URL=http://host.docker.internal:3000" >> /entrypoint.sh && \
   echo "ros2 launch vallet_cpp navigation_turtlebot3.launch.py" >> /entrypoint.sh

# Tornar o script executável
RUN chmod +x /entrypoint.sh

EXPOSE 3000

# Executa o lançamento
ENTRYPOINT ["/entrypoint.sh"]