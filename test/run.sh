#!/bin/sh

source devel/setup.sh
sleep 1

xterm -e roscore & 
sleep 1

xterm -e roslaunch src/app/launch/rtsp_stream_steering.launch &
sleep 1

xterm -e roslaunch src/app/launch/rtsp_stream_underbody.launch &
sleep 1

xterm -e rosrun image_view image_view image:=/cam_underbody/image_raw _image_transport:=compressed &
sleep 1

xterm -e rosrun image_view image_view image:=/cam_steering/image_raw _image_transport:=compressed &
sleep 1

#xterm -e python3 /srcapp/scripts/webstreaming.py &
#sleep 1

## Record all traffic
rosbag record 	/cam_underbody/image_raw/compressed \
				/cam_steering/image_raw/compressed
 

