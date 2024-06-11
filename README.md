# ROS2 CANOpen 4 DOF scara Control driver

This repository implements ROS2 control over ROS2 CANopen stack to control
4 DOF scara robot.
## OS installation
Ubuntu server 20.04
use Raspberry Pi Imager
Edit OS customization to enter wifi credentials and user credentials then click yes to proceed further
Power on Pi and wait for 5 mins. 

## Dependencies

- Docker (tested with version 24.0.6)
- Docker Compose (tested with version v2.21.0)

## Setup CAN interface 

Follow https://www.waveshare.com/wiki/RS485_CAN_HAT to setup the CAN interface.
####bcm2835
####Open the Raspberry Pi terminal and run the following command
wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.71.tar.gz
tar zxvf bcm2835-1.71.tar.gz 
cd bcm2835-1.71/
sudo ./configure && sudo make && sudo make check && sudo make install
####For more, you can refer to the official website at: http://www.airspayce.com/mikem/bcm2835/

####wiringPi
#Open the Raspberry Pi terminal and run the following command
cd
sudo apt-get install wiringpi
####For Raspberry Pi systems after May 2019 (earlier than that can be executed without), an upgrade may be required:
wget https://project-downloads.drogon.net/wiringpi-latest.deb
sudo dpkg -i wiringpi-latest.deb
gpio -v
#### Run gpio -v and version 2.52 will appear, if it doesn't it means there was an installation error

#### Bullseye branch system using the following command:
git clone https://github.com/WiringPi/WiringPi
cd WiringPi
. /build
gpio -v
#### Run gpio -v and version 2.70 will appear, if it doesn't it means there was an installation error

```bash
sudo modprobe vcan
sudo ip link add dev can0 type vcan
sudo ip link set can0 txqueuelen 1000
sudo ip link set up can0
candump can0 201:7ff  # 201:7ff to monitor position commands, 181:7FF for feedback
```
Replace can0 with can0 once real hardware is available and configured properly.

#### Clone repository
```bash
sudo apt update
sudo apt install -y git
git clone --recurse-submodules https://github.com/mehermadhu/4dof_scara.git
cd 4dof_scara

```

## Install Docker
```bash
sudo snap install docker
sudo usermod -aG docker $USER

```
### To Build Image if not built yet, to Create, start, and execute the container

For Building, creating and starting the container:

```bash
sudo docker compose up -d
```

### Run application

```bash
chmod +x launch_app.sh  # only first run

sudo docker compose exec ros2_humble bash
```
In the bash terminal of Docker run following commands
```
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

ros2 launch scara_4dof robot_control.launch.py

```

### Notes:
- To develop new controller do it inside can_open_tests package and build the whole package after mentioning the package name in CMakeLists file of canopen_tests package in the 
generate_dcf command.

- For remote desktop to share screen run command 
```
xhost +
```
in the remote computer. Not in docker terminal. 
 
 TODO:
 - xforward settings for client and host

### CAN driver setting
```
sudo nano /boot/config.txt
```
```
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000
```
### CAN bus setting
```
sudo ip link set can0 up type can bitrate 1000000
sudo ifconfig can0 txqueuelen 65536
reboot
```
### CAN driver verification

```
dmesg | grep -i '\(can\|spi\)'
```
### team viewer installation
```
sudo apt-get remove teamviewer
sudo apt-get purge teamviewer
sudo apt-get autoremove

wget https://download.teamviewer.com/download/linux/teamviewer-host_arm64.deb
sudo dpkg -i teamviewer-host_arm64.deb
sudo apt-get install -f -y
sudo teamviewer daemon start
systemctl enable teamviewerd
sudo reboot
```
### Add to startup applications
```
sh -c "sudo teamviewer --daemon start && teamviewer" on startup applications
```
### disable waynad
```
sudo nano /etc/gdm3/custom.conf
WaylandEnable=false
```

### for USB CAN of ch341-uart chip on Ubuntu 22.04 brl keyboard drivers will interfere
'''
sudo apt remove --purge brltty
'''