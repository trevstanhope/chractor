from chractor:latest

RUN /ros_entrypoint.sh
COPY ./resources/jca_j1939_to_ros.tar /tmp
RUN cd /tmp && tar -xvf /tmp/jca_j1939_to_ros.tar
RUN cd ~ && mkdir -p chractor/src && cd chractor/src && cp -r /tmp/jca_j1939_to_ros/ . && pwd && ls && cd ~
RUN /bin/bash -c 'source "/opt/ros/$ROS_DISTRO/setup.bash"; cd /root/chractor/; catkin_make'
COPY ./resources/opencvinfo.cc /root/
RUN cd /root/ && g++ opencvinfo.cc -l opencv_core -o opencvinfo
COPY ./resources/runserver.sh /
RUN chmod 755 /runserver.sh
ENTRYPOINT "/runserver.sh"

