# ROS2 CANOpen 4 DOF scara Control driver

This repository implements ROS2 control over ROS2 CANopen stack to control
4 DOF scara robot.

## Dependencies

- Docker (tested with version 24.0.6)
- Docker Compose (tested with version v2.21.0)

## Setup CAN interface 

Follow https://www.waveshare.com/wiki/RS485_CAN_HAT tosetup the CAN interface.

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 txqueuelen 1000
sudo ip link set up vcan0
```
Replace vcan0 with can0 once real hardware is available and configured properly.

## Build

### create project directory
```bash
mkdir scara_ctrl_project
```
### Clone this repository

```bash
cd scara_ctrl_project
git clone --recurse-submodules https://github.com/mehermadhu/4dof_scara.git'
```


### To Build Image if not built yet, to Create, start, and execute the container

For Building, creating and starting the container:

```bash
docker compose up -d
```

For executing commands in the container:

```bash
docker compose exec ros2_humble bash
```

### Run application
```bash
chmod +x launch_app.sh
./launch_app.sh
```

