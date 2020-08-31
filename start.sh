#!/bin/bash

ethtool -G enp4s0 rx 2048
modprobe peak_usb
ip link set can0 up type can bitrate 250000
ifconfig can0 up
ip link set can1 up type can bitrate 250000
ifconfig can1 up
ip link set can2 up type can bitrate 250000
ifconfig can1 up
source /opt/ros/kinetic/setup.bash
source catkin_ws/devel/setup.bash
source catkin_ws/devel/setup.sh
roslaunch sensor.launch
