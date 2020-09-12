#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Version 1.0

import os
import sys
import socket
import threading
import zmq
import logging
import yaml
import rospy

mutex = threading.Lock()
bigdata = bytearray()
log = None
yaml_import = None


def process_data():
    global bigdata
    global log

    with mutex:
        outdata = bigdata[:1777]
        bigdata = bigdata[1777:]

    #    log.debug("outdata %r", outdata)

    return outdata


def add_data(rawdata):
    global bigdata
    global log
    with mutex:
        bigdata.extend(rawdata)

    # log.debug("rawdata %r", rawdata)


def parse_packet(rawdata):
    version = (rawdata[0] & 0xC0) >> 6
    log.debug("RTP Version %d" % version)
    cc = rawdata[0] & 0x0F
    pktdata = rawdata[(12 + 4 * cc):]
    return pktdata


def udp_server(host='0.0.0.0', port=51060):
    global log
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.settimeout(2)

    rospy.init_node('rosj1939', anonymous=True)

    log.info("Listening on udp %s:%s" % (host, port))

    s.bind((host, port))
    while not rospy.is_shutdown():
        try:
            (data, addr) = s.recvfrom(128 * 1024)

            if data:
                b = bytearray()
                b.extend(data)
                add_data(parse_packet(b))
        except socket.timeout:
            continue


def convertvolt(data):
    b = float(data) * 1000
    c = int(b)
    msb = c / 256  # type: int
    lsb = c & 255  # type: int
    return lsb, msb


def readvolt(id):
    devname = "/dev/hbinvolt{}".format(id)
    try:
        infile = os.open(devname, os.O_RDONLY)
        v = os.read(infile, 16)
        os.close(infile)
    except OSError:
        v = "0.001"

    return convertvolt(v)


def ros_to_j1939():
    global log
    global yaml_import
    rospy.init_node('rosj1939', anonymous=True)
    rate = rospy.Rate(1)

    # Socket to talk to server
    context = zmq.Context()
    log.info("Connecting to the hummingbird")

    zs = context.socket(zmq.PUSH)
    zs.connect(yaml_import['J1939_ZMQ_SEND_SOCKET'])

    t = threading.Thread(target=udp_server)
    t.start()

    while not rospy.is_shutdown():
        data = bytearray([])
        data.append(yaml_import['J1939_PRIORITY'])
        data.append(yaml_import['J1939_PGN_MSB'])
        data.append(yaml_import['J1939_PGN_LSB'])
        data.append(0x00)  # Place holder, Source is determined by the stack

        (v0lsb, v0msb) = readvolt(0)
        data.append(v0lsb)
        data.append(v0msb)
        (v1lsb, v1msb) = readvolt(1)
        data.append(v1lsb)
        data.append(v1msb)
        (v2lsb, v2msb) = readvolt(2)
        data.append(v2lsb)
        data.append(v2msb)
        (v3lsb, v3msb) = readvolt(3)
        data.append(v3lsb)
        data.append(v3msb)

        data.extend(process_data())

        log.info("%d: %s", bigdata.__len__(), ''.join('{:02x}'.format(x) for x in data))

        zs.send(data)

        rate.sleep()

    zs.close()


if __name__ == '__main__':
    log = logging.getLogger('udp_server')

    try:
        FORMAT_CONS = '%(asctime)s %(name)-12s %(levelname)8s\t%(message)s'
        logging.basicConfig(level=logging.DEBUG, format=FORMAT_CONS)
        consolehandler = logging.StreamHandler(sys.stdout)
        log.addHandler(consolehandler)

        # Grab the socket connection from yaml
        with open(os.path.join(sys.path[0], "jca_j1939_to_ros.yaml"), 'r') as stream:
            try:
                yaml_import = yaml.load(stream)
                log.info("loaded socket from yaml: " + yaml_import['J1939_ZMQ_SEND_SOCKET'])
            except yaml.YAMLError as exc:
                pass

        ros_to_j1939()
    except rospy.ROSInterruptException:
        pass
