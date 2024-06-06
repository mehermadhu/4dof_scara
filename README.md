# ROS2 CANOpen 4 DOF scara Control driver

This repository implements ROS2 control over ROS2 CANopen stack to control
4 DOF scara robot.
## OS installation
Ubuntu server 20.04
use Raspberry Pi Imager
Edit OS customization then click yes to proceed further


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
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 txqueuelen 1000
sudo ip link set up vcan0
candump can0 201:7ff  # 201:7ff to monitor position commands, 181:7FF for feedback
```
Replace vcan0 with can0 once real hardware is available and configured properly.

#### Clone repository
```bash
git clone --recurse-submodules https://github.com/mehermadhu/4dof_scara.git'
cd 4dof_scara

```

## Install Docker
```bash
chmod +x docker_install.sh
./docker_install.sh
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
./launch_app.sh

```

