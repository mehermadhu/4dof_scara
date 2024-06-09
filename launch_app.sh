#!/bin/bash
sudo add-apt-repository -y universe
sudo apt update
sudo apt -y upgrade
sudo rosdep init 
rosdep update
rosdep install -i --from-paths src -y
colcon build
source install/setup.bash
# disable power saving and screen blank situations for ubuntu 21.04 onwards login with xorg desktop manager no waynad
xset s off
xset -dpms
xset s noblank
#ros2 launch scara_4dof robot_control.launch.py