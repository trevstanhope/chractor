#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import zmq
import rospy
import yaml

#Grab the socket connection from yaml
with open(os.path.join(sys.path[0], "jca_j1939_to_ros.yaml"), 'r') as stream:
    try:
        yaml_import = yaml.load(stream)
        J1939_ZMQ_SOCKET = yaml_import['J1939_ZMQ_SEND_SOCKET']
        print "loaded socket from yaml: " + J1939_ZMQ_SOCKET
    except yaml.YAMLError as exc:
        J1939_ZMQ_SOCKET = "tcp://127.0.0.1:5678"


# Socket to talk to server
context = zmq.Context()
print "Connecting to the hummingbird"

data = bytearray([])
data.append(0x18)
data.append(0xEF)
data.append(0x5A)
data.append(0x44)
data.append(0x11)
data.append(0x22)
data.append(0x33)
data.append(0x44)
data.append(0x55)
data.append(0x66)
data.append(0x77)
data.append(0x88)

zs = context.socket(zmq.PUSH)
zs.connect(J1939_ZMQ_SOCKET)


def ros_to_j1939():
    rospy.init_node('rosj1939', anonymous=True)

    zs.send(data)
    zs.close()

if __name__ == '__main__':
    try:
        ros_to_j1939()
    except rospy.ROSInterruptException:
        pass
