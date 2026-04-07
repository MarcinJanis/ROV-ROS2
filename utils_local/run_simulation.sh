#!/bin/bash
echo '--- Starting simulation ---';
# to build: 
# colcon build --symlink-install
source install/setup.bash

# export ROS_DOMAIN_ID=23

DISPLAY=:23 \
__NV_PRIME_RENDER_OFFLOAD=1 \
__GLX_VENDOR_LIBRARY_NAME=nvidia \
ros2 launch rov_stonefish rapture_bluerov2.launch.py \
scenario:=/home/dev/ros2_ws/src/simulation/rov_stonefish/scenarios/rapture.scn

DISPLAY=:23 \
__NV_PRIME_RENDER_OFFLOAD=1 \
__GLX_VENDOR_LIBRARY_NAME=nvidia \
ros2 launch rov_stonefish mediterranean1_bluerov2.launch.py \
scenario:=/home/dev/ros2_ws/src/simulation/rov_stonefish/scenarios/mediterranean1.scn


DISPLAY=:23 \
__NV_PRIME_RENDER_OFFLOAD=1 \
__GLX_VENDOR_LIBRARY_NAME=nvidia \
ros2 launch rov_stonefish fiords_bluerov2.launch.py \
scenario:=/home/dev/ros2_ws/src/simulation/rov_stonefish/scenarios/fiords.scn

DISPLAY=:23 \
__NV_PRIME_RENDER_OFFLOAD=1 \
__GLX_VENDOR_LIBRARY_NAME=nvidia \
ros2 launch rov_stonefish longisland_bluerov2.launch.py \
scenario:=/home/dev/ros2_ws/src/simulation/rov_stonefish/scenarios/longisland_bluerov2.scn


# DISPLAY=:23 
# __NV_PRIME_RENDER_OFFLOAD=1 \
# __GLX_VENDOR_LIBRARY_NAME=nvidia \
# ros2 launch rov_stonefish mediterranean1_bluerov2.launch.py \
# scenario:=/home/dev/ros2_ws/src/simulation/rov_stonefish/scenarios/mediterranean1.scn

# DISPLAY=:23 \
# __NV_PRIME_RENDER_OFFLOAD=1 \
# __GLX_VENDOR_LIBRARY_NAME=nvidia \
# ros2 launch rov_stonefish mediterranean2_bluerov2.launch.py \
# scenario:=/home/dev/ros2_ws/src/simulation/rov_stonefish/scenarios/mediterranean2.scn



# sudo chmod 666 /dev/nvidia*
# --- in docker --- 
# In case of problem with acces to gpu:
# - sudo hase rights to use gpu 
# sudo groupadd -g 1001 nvidia_host
# sudo usermod -aG nvidia_host dev
# newgrp nvidia_host
# nvidia-smi

