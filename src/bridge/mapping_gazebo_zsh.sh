#!/bin/zsh

echo "Updating packages..."
# Update packages
sudo apt-get update -y > /dev/null 2>&1

echo "Installing xterm..."
# Install xterm
sudo apt-get install -y xterm > /dev/null 2>&1

echo "Building with colcon..."
# Build with colcon
colcon build > /dev/null 2>&1

echo "Configuring environment..."
# Configure environment
source install/local_setup.zsh

echo "Runing the launch file..."
# Launch the launch file
ros2 launch mapping_gazebo mapping_gazebo.launch.py
