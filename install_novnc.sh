#!/bin/bash

# Install VNC server and other necessary packages
sudo apt update
sudo apt install -y x11vnc xvfb fluxbox

# Setup VNC server to start with the system
x11vnc -storepasswd your_vnc_password_here /etc/x11vnc.pass
sudo bash -c 'echo "[Unit]
Description=Start x11vnc at startup.
After=multi-user.target

[Service]
Type=simple
ExecStart=/usr/bin/x11vnc -auth guess -forever -loop -noxdamage -repeat -rfbauth /etc/x11vnc.pass -rfbport 5900 -shared

[Install]
WantedBy=multi-user.target" > /etc/systemd/system/x11vnc.service'

# Enable the VNC server service so it starts on boot
sudo systemctl enable x11vnc.service
sudo systemctl start x11vnc.service

# Install noVNC
cd /opt
sudo git clone https://github.com/novnc/noVNC.git
sudo git clone https://github.com/novnc/websockify noVNC/utils/websockify


sudo openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout /opt/noVNC/utils/websockify/self.pem -out /opt/noVNC/utils/websockify/self.pem


sudo nano /etc/systemd/system/novnc.service

[Unit]
Description=noVNC Service
After=network.target

[Service]
Type=simple
User=root
ExecStart=/bin/bash -c '/opt/noVNC/utils/novnc_proxy --vnc localhost:5901 --cert /opt/noVNC/utils/websockify/self.pem'

[Install]
WantedBy=multi-user.target

sudo systemctl start novnc.service

sudo systemctl status novnc.service

sudo systemctl enable novnc.service

sudo journalctl -u novnc.service

sudo nano /etc/systemd/system/x11vnc.service

[Unit]
Description=Start x11vnc at startup
After=multi-user.target

[Service]
Type=simple
ExecStart=/usr/bin/x11vnc -rfbport 5901 -forever -auth guess

[Install]
WantedBy=multi-user.target


sudo systemctl enable x11vnc.service
sudo systemctl start x11vnc.service
 