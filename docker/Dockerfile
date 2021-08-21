from chractor:latest

RUN /ros_entrypoint.sh
COPY ./resources/app.tar /tmp
RUN cd /tmp && tar -xvf /tmp/app.tar
RUN cd ~ && mkdir -p chractor/src && cd chractor/src && cp -r /tmp/app/ . && pwd && ls && cd ~
# ROS not currently supported
# RUN /bin/bash -c 'source "/opt/ros/$ROS_DISTRO/setup.bash"; cd /root/chractor/; catkin_make'
# COPY ./resources/opencvinfo.cc /root/
# RUN cd /root/ && g++ opencvinfo.cc -l opencv_core -o opencvinfo
COPY ./resources/runserver.sh /
COPY ./resources/install.sh /
RUN chmod 755 /runserver.sh
RUN chmod 755 /install.sh
ENTRYPOINT "/runserver.sh"

