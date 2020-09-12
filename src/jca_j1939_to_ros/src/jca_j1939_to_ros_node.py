#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import subprocess
import zmq
import rospy
from jca_j1939_to_ros.msg import j1939
import yaml

J1939_ZMQ_SOCKET_DEFAULT = 'tcp://127.0.0.1:5678'
J1939_NODE_DEFAULT = 'J1939_raw'
J1939_TOPIC_DEFAULT = 'J1939_raw'
J1939_PGN_LIST_DEFAULT = [format(i, '04X') for i in range(0, 0x10000)]

def send_msg_to_syslog(msg, level):
    msg = "[%s][%s][%s][%s]: "%("JCA","J1939", level, sys.argv[0].split('/')[-1]) + msg
    child = subprocess.Popen(["logger",msg])
    streamdata = child.communicate()[0]
    return child.returncode

def load_key(dictionary, key, default):
    try:
        return_val = dictionary[key]
    except KeyError:
        return_val = default
    return return_val

def j1939_to_ros():
    #Grab the socket connection from yaml
    try:
        with open(os.path.join(sys.path[0], "jca_j1939_to_ros.yaml"), 'r') as stream:
            try:
                yaml_import = yaml.load(stream)
            except yaml.YAMLError as exc:
                send_msg_to_syslog("Could load the yaml file", "ERROR")
                raise exc
    except IOError:
        send_msg_to_syslog("Yaml file does not exsist.", "WARNING")
        yaml_import = dict()

    J1939_ZMQ_SOCKET = load_key(yaml_import, 'J1939_ZMQ_RECV_SOCKET', J1939_ZMQ_SOCKET_DEFAULT)
    J1939_NODE = load_key(yaml_import, 'J1939_NODE', J1939_NODE_DEFAULT)
    J1939_TOPIC = load_key(yaml_import, 'J1939_TOPIC', J1939_TOPIC_DEFAULT)
    J1939_PGN_LIST_STR = load_key(yaml_import, 'J1939_PGN_LIST', J1939_PGN_LIST_DEFAULT)
    if(type(J1939_PGN_LIST_STR) is not list):
        J1939_PGN_LIST_STR = J1939_PGN_LIST_DEFAULT
    else:
        J1939_PGN_LIST_STR = list([s.upper() for s in J1939_PGN_LIST_STR])

    J1939_PGN_LIST = dict(zip(J1939_PGN_LIST_STR, [int(i, 16) for i in J1939_PGN_LIST_STR]))

    rospy.init_node(J1939_NODE)
    pub = rospy.Publisher(J1939_TOPIC, j1939, queue_size=10)
    j1939_msg = j1939()
    # Socket to talk to server
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect(J1939_ZMQ_SOCKET)

    socket.setsockopt(zmq.SUBSCRIBE, '')

    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)

    send_msg_to_syslog("The J1939 to ROS node is online", "INFO")

    while not rospy.is_shutdown():
        message_bytes = bytearray()
        socks = dict(poller.poll(250)) #waits 250ms before timing out
        if socks:
            if socks.get(socket) == zmq.POLLIN:
                message = socket.recv_multipart()
                message_bytes.extend(message[1])
                pgn = (message_bytes[1] << 8 | message_bytes[2]) & 0xffff
                #print  "%0.4X" %  1243 + "   " + "%0.4X" %  pgn in J1939_PGN_LIST
                if "%0.4X" %  pgn in J1939_PGN_LIST:
                    j1939_msg.id = list(message_bytes[0:4])
                    j1939_msg.data = list(message_bytes[4:])
                    pub.publish(j1939_msg)

if __name__ == '__main__':
    try:
        j1939_to_ros()
    except rospy.ROSInterruptException: pass

