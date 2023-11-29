#!/bin/bash

echo "Updating packages..."
sudo apt-get update -y > /dev/null 2>&1

echo "Installing xterm, tf-transformations and others auxiliar packages..."
sudo apt-get install -y xterm > /dev/null 2>&1
sudo apt install -y ros-humble-tf-transformations > /dev/null 2>&1
sudo apt install -y python3-pip python3-virtualenv > /dev/null 2>&1

echo "Creating virtual environment and installing pip requeriments..."
virtualenv -p python3 ./venv > /dev/null 2>&1
source ./venv/bin/activate > /dev/null 2>&1
touch ./venv/COLCON_IGNORE > /dev/null 2>&1
pip install -r requirements.txt > /dev/null 2>&1

echo "Building with colcon..."
colcon build > /dev/null 2>&1

echo "Configuring environment..."
source install/local_setup.bash
export $1

echo "Runing the launch file..."
ros2 launch vallet navigation_turtlebot3.launch.py
