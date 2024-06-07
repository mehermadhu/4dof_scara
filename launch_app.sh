#!/bin/bash
sudo add-apt-repository -y universe

sudo systemctl set-default graphical.target
#sudo apt install -y xserver-xorg  # not required for ubuntu 20.03
sudo apt-get install -y x11-xserver-utils
sudo apt update
sudo apt -y upgrade
sudo rosdep init 
rosdep update
rosdep install -i --from-paths src -y
colcon build
source install/setup.bash
ros2 launch scara_4dof robot_control.launch.py