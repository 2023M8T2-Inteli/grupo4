#!/bin/bash

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
source install/local_setup.bash

echo "Runing the launch file..."
# Launch the launch file
ros2 launch navigation_turtlebot3 navigation_turtlebot3.launch.py
