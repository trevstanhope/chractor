iosbag record \
/right/scan \
/rtsp_1/image_raw \
/rtsp_2/image_raw \
/can0/received_messages \
/can1/received_messages \
/can2/received_messages \
-b 8000 \
-o data.bag --size=4000 --split
