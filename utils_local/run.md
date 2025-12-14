

# Commands
___

> build: `colcon build --symlink-install` 
> source: `source install/setup.bash`

> private channels `export ROS_DOMAIN_ID=23` (in each terminal)

> run simulation:
`DISPLAY=:23 \
__NV_PRIME_RENDER_OFFLOAD=1 \
__GLX_VENDOR_LIBRARY_NAME=nvidia \
ros2 launch rov_stonefish rapture_bluerov2.launch.py \
scenario:=/home/dev/ros2_ws/src/simulation/rov_stonefish/scenarios/rapture.scn`

> run control panel:
`python3 ./src/simulation/rov_stonefish/odometry_getdata/control_panel.py`