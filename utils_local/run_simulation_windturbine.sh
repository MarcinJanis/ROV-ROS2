#!/bin/bash
echo '--- Starting simulation ---';
# to build: 
# colcon build --symlink-install
source install/setup.bash

# DISPLAY=:23 \
# __NV_PRIME_RENDER_OFFLOAD=1 \
# __GLX_VENDOR_LIBRARY_NAME=nvidia \
# ros2 launch rov_stonefish windturbine_bluerov2.launch.py \
# scenario:=/home/dev/ros2_ws/src/simulation/rov_stonefish/scenarios/windturbine_bluerov2.scn

DISPLAY=:23 \
__NV_PRIME_RENDER_OFFLOAD=1 \
__GLX_VENDOR_LIBRARY_NAME=nvidia \
ros2 launch rov_stonefish rapture_bluerov2.launch.py \
scenario:=/home/dev/ros2_ws/src/simulation/rov_stonefish/scenarios/rapture_bluerov2.scn




# In case of problem with acces to gpu:
# - sudo hase rights to use gpu 
# sudo groupadd -g 1001 nvidia_host
# sudo usermod -aG nvidia_host dev
# newgrp nvidia_host
# nvidia-smi

