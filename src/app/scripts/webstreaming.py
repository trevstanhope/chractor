"""
Webstreaming

Supports:
* 2x Ethernet RTP cameras

In Development
* 1x CAN 3-axis gyroscope
* 4x Analog voltage sensor

"""

__author__      = "Trevor Stanhope"
__copyright__   = "MIT"
__date__ = "2021-04-11"

# import the necessary packages
from imutils.video import VideoStream
from flask import Response
from flask import Flask
from flask import render_template
import threading
import argparse
import datetime
import imutils
import time
import cv2
import numpy as np
import socket
import os
import logging
import sys
#import yaml
#import zmq

# constants
DEBUG = False
VIDEO_HEIGHT = 240
VIDEO_WIDTH = 320
HOST = "0.0.0.0"
PORT = 8000
FRAMERATE = 5
BOOT_DELAY = 2.0

# Globals
logging.basicConfig(level=logging.WARNING, filename="/tmp/chractor.log")
log = logging.getLogger(__name__)
consolehandler = logging.StreamHandler(sys.stdout)
log.addHandler(consolehandler)
log.info("initiating globals ...")
outputFrame_steering = np.zeros((VIDEO_HEIGHT,VIDEO_WIDTH,3), np.uint8)
outputFrame_underbody = np.zeros((VIDEO_HEIGHT,VIDEO_WIDTH,3), np.uint8)
lock = threading.Lock()
app = Flask(__name__) # initialize a flask object

# initialize the video stream and allow the camera sensor to  warmup
log.info("initiating streams ...")
try:
	vs_steering = VideoStream("rtp://192.168.40.181:50008").start() #!TODO: Create centralized config for this (SHARED WITH LAUNCH XMLS)
	log.info("started steering stream")
except:
	log.warning("failed to start steering stream")
	vs_steering = None

# initialize the video stream and allow the camera sensor to  warmup
try:
	vs_underbody = VideoStream("rtp://192.168.40.40:50004").start() #!TODO: Create centralized config for this (SHARED WITH LAUNCH XMLS)
	log.info("started underbody stream")
except:
	log.warning("failed to start underbody stream")
	vs_underbody = None
time.sleep(BOOT_DELAY)


# Functions
def capture_streams():
	# capture video streams

	# grab global references to the video stream(s), output frame, and
	# lock variables
	global vs_steering, vs_underbody, outputFrame_steering, outputFrame_underbody, lock

	# loop over frames from the video stream
	while True:
		time.sleep(1.0/FRAMERATE)
		# read the next frame from the video stream, resize it,
		if vs_steering is not None: 
			frame_steering = vs_steering.read()
			if frame_steering is not None:
				frame_steering = imutils.resize(frame_steering, width=VIDEO_WIDTH, height=VIDEO_HEIGHT)
				with lock:
					outputFrame_steering = frame_steering.copy()
			else:
				outputFrame_steering = np.zeros((VIDEO_HEIGHT,VIDEO_WIDTH,3), np.uint8)
		
		# read the next frame from the video stream, resize it,
		if vs_underbody is not None: 
			frame_underbody = vs_underbody.read()
			if frame_underbody is not None:
				frame_underbody = imutils.resize(frame_underbody, width=VIDEO_WIDTH, height=VIDEO_HEIGHT)
				with lock:
					outputFrame_underbody = frame_underbody.copy()
			else:
				 outputFrame_underbody = np.zeros((VIDEO_HEIGHT,VIDEO_WIDTH,3), np.uint8)

def generate_display():	
	# generate display graphic for web UI

	# grab global references to the output frame and lock variables
	global outputFrame_steering, outputFrame_underbody, lock
	# loop over frames from the output stream
	while True:
		# wait until the lock is acquired
		with lock:
			# check if the output frame is available, otherwise skip
			# the iteration of the loop
			outputFrame_combo = np.hstack((outputFrame_steering, outputFrame_underbody))
			# encode the frame in JPEG format
			(flag, encodedImage_combo) = cv2.imencode(".jpg", outputFrame_combo)
			# ensure the frame was successfully encoded
			if not flag:
				continue
		
		# yield the output frame in the byte format
		yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage_combo) + b'\r\n')

def capture_can(host='0.0.0.0', port=51060):
	# UDP server to capture UDP CAN packets

    global log
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.settimeout(2)

    log.info("Listening on udp %s:%s" % (host, port))

    s.bind((host, port))
    while True:
        try:
            (data, addr) = s.recvfrom(128 * 1024)
            if data:
                b = bytearray()
                b.extend(data)
               	parse_packet(b)
        except socket.timeout:
            continue

def parse_packet(rawdata):
	# parse UDP CAN packet

    version = (rawdata[0] & 0xC0) >> 6
    log.debug("RTP Version %d" % version)
    cc = rawdata[0] & 0x0F
    pktdata = rawdata[(12 + 4 * cc):]
    return pktdata

def read_volts(id):
	# read volts from device

    devname = "/dev/hbinvolt{}".format(id)
    try:
        infile = os.open(devname, os.O_RDONLY)
        v = os.read(infile, 16)
        os.close(infile)
    except OSError:
        v = "0.001"

def capture_volts():

	while True:
		try:
			v0 = read_volts(0)
			v1 = read_volts(1)
			v2 = read_volts(2)
			v3 = read_volts(3)
		except Exception as E:
			print("err")
 
## Flask Handlers
@app.route("/")
def index():

	# return the rendered template
	return render_template("index.html")

@app.route("/video_feed")
def video_feed():
	# return the response generated along with the specific media
	# type (mime type)
	return Response(generate_display(), mimetype = "multipart/x-mixed-replace; boundary=frame")

# check to see if this is the main thread of execution
if __name__ == '__main__':

	# construct the argument parser and parse command line arguments
	#ap = argparse.ArgumentParser()
	#ap.add_argument("-i", "--ip", type=str, required=True, help="ip address of the device")
	#ap.add_argument("-o", "--port", type=int, required=True, help="ephemeral port number of the server (1024 to 65535)")
	#ap.add_argument("-f", "--frame-count", type=int, default=32, help="# of frames used to construct the background model")
	#args = vars(ap.parse_args())
	
	#Socket to talk to server
	#context = zmq.Context()
	#log.info("connecting to the hummingbird ...")
	#zs = context.socket(zmq.PUSH)
	#zs.connect(yaml_import['J1939_ZMQ_SEND_SOCKET'])

	# start a thread that will perform stream capture
	#t = threading.Thread(target=capture_streams, args=(args["frame_count"],))
	log.info("initiating capture streams thread ...")
	t_streams = threading.Thread(target=capture_streams)
	t_streams.daemon = True
	t_streams.start()

	# start a thread that will perform UDP CAN capture
	log.info("initiating UDP CAN server thread ...")
	t_can = threading.Thread(target=capture_can)
	t_can.daemon = True
	t_can.start()

	# start a thread that will perform voltage readings
	log.info("initiating UDP CAN server thread ...")
	t_volts = threading.Thread(target=capture_volts)
	t_volts.daemon = True
	t_volts.start()

	# start the flask app
	#app.run(host=args["ip"], port=args["port"], debug=DEBUG, threaded=True, use_reloader=False)
	log.info("initiating web app ...")
	app.run(host=HOST, port=PORT, debug=DEBUG, threaded=True, use_reloader=False)

# release the video stream pointer
if vs_steering is not None:
	log.info("releasing steering stream ...")
	vs_steering.stop()

# release the video stream pointer
if vs_underbody is not None:
	log.info("releasing underbody stream ...")
	vs_underbody.stop()
