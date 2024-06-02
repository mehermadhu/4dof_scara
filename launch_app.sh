#!/bin/bash
sudo add-apt-repository -y universe
sudo apt update
sudo apt -y upgrade
sudo rosdep init 
rosdep update
rosdep install -i --from-paths src -y
colcon build
source install/setup.bash
ros2 launch scara_4dof robot_control.launch.py