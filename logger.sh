#!/bin/bash

## Init Devices
## NOTE Comment if you do not have these set to automatically initialize on boot
ethtool -G enp4s0 rx 2048
#modprobe peak_usb
#ip link set can0 up type can bitrate 250000
#ifconfig can0 up
#ip link set can1 up type can bitrate 500000
#ifconfig can1 up

# Init ROS Workspace
source /opt/ros/kinetic/setup.bash
source devel/setup.sh

# Init ROS Topics (e.g. CAN and Video streams)
# NOTE:  opens a second terminal window where  ROS diagnostics messaging is
# displayed. If any REDTEXT or warnings appear, ensure that all ROS topics
# have successfully launched before continuing with data collection!
xterm -e roslaunch app rtsp_stream_steering.launch &
xterm -e roslaunch app rtsp_stream_undercarriage.launch &
xterm -e rosrun image_view image_view image:=/cam_steering/image_raw compressed &

rosbag record -O data/test.bag \
	/cam_steering/image_raw/compressed \
	/can0/received_messages \
	/can1/received_messages \
	/rosout \
	/rosout_agg \
	-b 8000 \
	--size=4000 \
	--split

# Ctrl-C to exit
