#!/bin/bash

# Update and install locales
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Install necessary packages
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Update and install curl
sudo apt update && sudo apt install -y curl

# Add the ROS repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and upgrade packages
sudo apt update
sudo apt upgrade -y

# Install ROS Humble base, additional ROS tools, and other useful packages
sudo apt install -y \
    ros-humble-ros-base \
    ros-humble-rviz2 \
    ros-humble-rqt* \
    ros-dev-tools \
    python3-rosdep \
    python3-colcon-common-extensions \
    can-utils \
    net-tools \
    nano

# Initialize rosdep if you haven't done so
sudo rosdep init
rosdep update

# Check for and install any missing dependencies for all installed packages
rosdep install --from-paths /opt/ros/humble/share --ignore-src --rosdistro humble -y

# Handle possible missing library for rviz2
sudo apt-get install -y libogre-1.12-dev
sudo apt-get install --reinstall ros-humble-rviz2

# Add ROS environment to bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Ensure the environment is properly loaded
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

echo "ROS Humble and dependencies have been installed and configured, along with additional checks for rviz2 compatibility."
