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

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 txqueuelen 1000
sudo ip link set up vcan0
candump can0 201:7ff  # 201:7ff to monitor position commands, 181:7FF for feedback
```
Replace vcan0 with can0 once real hardware is available and configured properly.

### Clone repository
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

