#!/bin/bash
gnome-terminal --title="Listener" -- bash -c "

echo 'Starting listener'; 

source install/setup.bash;
cd ./src/simulation/rov_stonefish/odometry_getdata;
python3 ros_listener.py;

sleep 5; 
exec bash"

# In case of problem with acces to gpu:
# - sudo hase rights to use gpu 
# sudo groupadd -g 1001 nvidia_host
# sudo usermod -aG nvidia_host dev
# newgrp nvidia_host
# nvidia-smi

