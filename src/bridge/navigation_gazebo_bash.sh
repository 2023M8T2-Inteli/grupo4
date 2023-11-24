#!/bin/bash

echo "Updating packages..."
# Update packages
sudo apt-get update -y > /dev/null 2>&1

echo "Installing xterm, tf-transformations and others auxiliar packages..."
# Install xterm and tf-transformations
sudo apt-get install -y xterm > /dev/null 2>&1
sudo apt install ros-humble-tf-transformations > /dev/null 2>&1
sudo apt install python3-pip > /dev/null 2>&1
pip install -r requirements.txt > /dev/null 2>&1

echo "Building with colcon..."
# Build with colcon
colcon build > /dev/null 2>&1

echo "Configuring environment..."
# Configure environment
source install/local_setup.bash

echo "Runing the launch file..."
# Launch the launch file
ros2 launch navigation_gazebo navigation_gazebo.launch.py
