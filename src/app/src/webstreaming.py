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
__date__ = "2021-08-19"

# Import necessary packages
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
import zmq
import random

## Constants
DEBUG = False
VIDEO_HEIGHT = 240
VIDEO_WIDTH = 320
HOST = "0.0.0.0"
PORT = 8000
FRAMERATE = 5
BOOT_DELAY = 2.0
J1939_ZMQ_RECV_SOCKET = 'tcp://127.0.0.1:5678'
J1939_ZMQ_SEND_SOCKET = 'tcp://127.0.0.1:5555'
MTLT305D_ANGLES_PGN = 0xF029
MTLT305D_ACCEL_PGN = 0xF02D
MTLT305D_ANGLERATE_PGN = 0xF02A
J1939_PGN_LIST = [MTLT305D_ANGLES_PGN, MTLT305D_ACCEL_PGN, MTLT305D_ANGLERATE_PGN]
ANGLE_GREEN = 0
ANGLE_YELLOW = 15
ANGLE_ORANGE = 30
ANGLE_RED = 45
STEERING_RTP_ADDR = "rtp://192.168.40.181:50008"
UNDERBODY_RTP_ADDR = "rtp://192.168.40.40:50004"

## Globals
logging.basicConfig(level=logging.DEBUG, filename="/tmp/chractor.log")
log = logging.getLogger(__name__)
consolehandler = logging.StreamHandler(sys.stdout)
log.addHandler(consolehandler)
log.info("initiating globals ...")
outputFrame_steering = np.zeros((VIDEO_HEIGHT,VIDEO_WIDTH,3), np.uint8)
outputFrame_underbody = np.zeros((VIDEO_HEIGHT,VIDEO_WIDTH,3), np.uint8)
lock = threading.Lock()
app = Flask(__name__) # initialize a flask object
gyrox = 0
gyroy = 0
gyroz = 0
accelx = 0
accely = 0
accelz = 0
pitch = 0
roll = 0
v0 = 0
v1 = 0
v2 = 0
v3 = 0

## Classes
class j1939_msg:
    data = bytearray(8)
    id = bytearray(4)

## Functions
def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

def MTLT305D_accel_handler(msg):

	id = 0xCF02D80
	pgn = 0xF02D
	sa = 0x80
	da = 0xFF
	dlc = 8
	data = {
		'accelx_uint16' : (msg.data[0] << 8 | msg.data[1]) & 0xffff,
		'accely_uint16' : (msg.data[2] << 8 | msg.data[3]) & 0xffff,
		'accelz_uint16' : (msg.data[4] << 8 | msg.data[5]) & 0xffff,
		'lateralacc_figure_ofmerit_int2' : msg.data[6] >> 0 & 0b11,
		'longiacc_figure_ofmerit_int2' : msg.data[6] >> 2 & 0b11,
		'vertiacc_figure_ofmerit_int2' : msg.data[6] >> 4 & 0b11,
		'support_rate_acc_int8' : msg.data[7]
	}
	factor = 0.1
	offset = -320
	vmin = -320
	vmax = 335.35
	accelx = clamp(data['accelx_uint16'] * factor + offset, vmin, vmax) # m/s^2
	accely = clamp(data['accely_uint16'] * factor + offset, vmin, vmax) # m/s^2
	accelz = clamp(data['accelz_uint16'] * factor + offset, vmin, vmax) # m/s^2
	return accelx, accely, accelz

def MTLT305D_angles_handler(msg):

	pgn = 0xF029
	sa = 0x80
	da = 0xFF
	dlc = 8
	data = {
		'pitch_uint24' : msg.data[0] | (msg.data[1] << 8) | (msg.data[2] << 16) & 0xffffff,
		'roll_uint24' : msg.data[3] | (msg.data[4] << 8) | (msg.data[5] << 16) & 0xffffff,
		'pitch_compensation_int2' : msg.data[6] >> 0 & 0b11,
		'pitch_figure_ofmerit_int2' : msg.data[6] >> 2 & 0b11,
		'roll_compensation_int2' : msg.data[6] >> 4 & 0b11,
		'roll_figure_ofmerit_int2' : msg.data[6] >> 6 & 0b11,
		'pitchroll_latency_int8' : msg.data[7]
	}
	factor = 3.0517578125e-05
	offset = -250
	vmin = -90 # default is -250
	vmax = 90 # default is 252
	pitch = clamp(data['pitch_uint24'] * factor + offset, vmin, vmax) # deg
	roll = clamp(data['roll_uint24'] * factor + offset, vmin, vmax) # deg
	return pitch, roll

def MTLT305D_anglerate_handler(msg):

	pgn = 0xF029
	sa = 0x80
	da = 0xFF
	dlc = 8
	data = {
		'gyrox_uint16' : (msg.data[0] << 8 | msg.data[1]) & 0xffff,
		'gyroy_uint16' : (msg.data[2] << 8 | msg.data[3]) & 0xffff,
		'gyroz_uint16' : (msg.data[4] << 8 | msg.data[5]) & 0xffff,
		'pitchrate_figure_ofmerit_int2' : msg.data[6] >> 0 & 0b11,
		'rollrate_figure_ofmerit_int2' : msg.data[6] >> 2 & 0b11,
		'yawrate_figure_ofmerit_int2' : msg.data[6] >> 4 & 0b11,
		'anglerate_latency_int8' : msg.data[7],
	}
	factor = 0.0078125
	offset = -250
	vmin = -250
	vmax = 261.9921875
	gyrox = clamp(data['gyrox_uint16'] * factor + offset, vmin, vmax) # deg/s
	gyroy = clamp(data['gyroy_uint16'] * factor + offset, vmin, vmax) # deg/s
	gyroz = clamp(data['gyroz_uint16'] * factor + offset, vmin, vmax) # deg/s
	return gyrox, gyroy, gyroz

def capture_j1939():
	# Socket to talk to J1939 Stack
	global gyrox, gyroy, gyroz
	global pitch, roll
	global accelx, accely, accelz

    # Socket to Receive
	log.info("initiating ZMQ RECV socket ...")
	context = zmq.Context()
	socket_recv = context.socket(zmq.SUB)
	socket_recv.connect(J1939_ZMQ_RECV_SOCKET)
	socket_recv.setsockopt_string(zmq.SUBSCRIBE, '')
	poller = zmq.Poller()
	poller.register(socket_recv, zmq.POLLIN)
	log.info("ZMQ RECV socket open ...")
	
	# Socket to Send
#	socket_send = context.socket(zmq.PUSH)
#	socket_send.connect(J1939_ZMQ_SEND_SOCKET)
#	log.info("ZMQ SEND socket open ...")
#	data = bytearray([])
#	data.append(0x18)
#	data.append(0xEF)
#	data.append(0x5A)
#	data.append(0x44)
#	data.append(0x11)
#	data.append(0x22)
#	data.append(0x33)
#	data.append(0x44)
#	data.append(0x55)
#	data.append(0x66)
#	data.append(0x77)
#	data.append(0x88)
#   socket_send.send(data)

	while not False:
		message_bytes = bytearray()
		socks = dict(poller.poll(250)) # waits 250ms before timing out
		if socks:
			if socks.get(socket) == zmq.POLLIN:
				message = socket.recv_multipart()
				message_bytes.extend(message[1])
				pgn = (message_bytes[1] << 8 | message_bytes[2]) & 0xffff
				if pgn in J1939_PGN_LIST:
					j1939_msg.id = list(message_bytes[0:4])
					j1939_msg.data = list(message_bytes[4:])
					if pgn == MTLT305D_ACCEL_PGN:
						accelx, accely, accelz = MTLT305D_accel_handler(j1939_msg)
					if pgn == MTLT305D_ANGLES_PGN:
						pitch, roll = MTLT305D_angles_handler(j1939_msg)
					if pgn == MTLT305D_ANGLERATE_PGN:
						gyrox, gyroy, gyroz = MTLT305D_anglerate_handler(j1939_msg)

	socket_recv.close()
	#socket_send.close()

def capture_streams():
	# capture RTP video streams

	# grab global references to the video stream(s), output frame, and
	# lock variables
	global cam_steering, cam_underbody, outputFrame_steering, outputFrame_underbody, lock

	# loop over frames from the video stream
	while True:
		time.sleep(1.0/FRAMERATE)
		# read the next frame from the video stream, resize it,
		if cam_steering is not None: 
			frame_steering = cam_steering.read()
			if frame_steering is not None:
				frame_steering = imutils.resize(frame_steering, width=VIDEO_WIDTH, height=VIDEO_HEIGHT)
				with lock:
					outputFrame_steering = frame_steering.copy()
			else:
				outputFrame_steering = np.zeros((VIDEO_HEIGHT,VIDEO_WIDTH,3), np.uint8)
				cv2.putText(
					 outputFrame_steering, #numpy array on which text is written
					 str("NO CAM"), #text
					 (int(VIDEO_WIDTH/3), int(VIDEO_HEIGHT/2)), # position at which writing has to start
					 cv2.FONT_HERSHEY_SIMPLEX, #font family
					 1, #font size
					 (255,255,255), #font color
					 3 #font stroke
				)
		
		# read the next frame from the video stream, resize it,
		if cam_underbody is not None: 
			frame_underbody = cam_underbody.read()
			if frame_underbody is not None:
				frame_underbody = imutils.resize(frame_underbody, width=VIDEO_WIDTH, height=VIDEO_HEIGHT)
				with lock:
					outputFrame_underbody = frame_underbody.copy()
			else:
				outputFrame_underbody = np.zeros((VIDEO_HEIGHT,VIDEO_WIDTH,3), np.uint8)
				cv2.putText(
					 outputFrame_underbody, #numpy array on which text is written
					 str("NO CAM"), #text
					 (int(VIDEO_WIDTH/3),int(VIDEO_HEIGHT/2)), # position at which writing has to start
					 cv2.FONT_HERSHEY_SIMPLEX, #font family
					 1, #font size
					 (255,255,255), #font color
					 3 #font stroke
				)

def color_picker(val):
	if abs(val) < ANGLE_YELLOW: return (0, 255, 0)
	elif abs(val) < ANGLE_ORANGE: return (0, 255, 255)
	elif abs(val) < ANGLE_RED: return (0, 128, 255)
	else: return (0, 0, 255)
		
def generate_display():	
	# generate display graphic for web UI

	# grab global references to the output frame and lock variables
	global outputFrame_steering, outputFrame_underbody, lock
	global roll, pitch
	global accelx, accely, accelz
	global gyrox, gyroy, gyroz

	# loop over frames from the output stream
	while True:
		# wait until the lock is acquired
		with lock:
			# check if the output frame is available, otherwise skip
			# the iteration of the loop
			outputFrame_combo = np.hstack((outputFrame_steering, outputFrame_underbody))
			color = color_picker(roll)
			cv2.putText(
				 outputFrame_combo, #numpy array on which text is written
				 str("roll: %+05.1f" % roll), #text
				 (10,30), # position at which writing has to start
				 cv2.FONT_HERSHEY_SIMPLEX, #font family
				 1, #font size
				 color, #font color
				 3 #font stroke
			)
			color = color_picker(pitch)
			cv2.putText(
				 outputFrame_combo, #numpy array on which text is written
				 str("pitch: %+05.1f" % pitch), #text
				 (430,30), # position at which writing has to start
				 cv2.FONT_HERSHEY_SIMPLEX, #font family
				 1, #font size
				 color, # font color
				 3 #font stroke
			)			
			# encode the frame in JPEG format
			(flag, encodedImage_combo) = cv2.imencode(".jpg", outputFrame_combo)
			# ensure the frame was successfully encoded
			if not flag:
				continue
		
		# yield the output frame in the byte format
		yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage_combo) + b'\r\n')

def read_volts(id):
	# read volts from device
	devname = "/dev/hbinvolt{}".format(id)
	try:
		infile = os.open(devname, os.O_RDONLY)
		v = float(os.read(infile, 16))
		os.close(infile)
	except OSError:
		log.info("couldn't read ADIO device")
		v = 0.000
	return v

def capture_adio():
	global v0, v1, v2, v3
	while True:
		try:
			v0 = read_volts(0)
			v1 = read_volts(1)
			v2 = read_volts(2)
			v3 = read_volts(3)
			log.info("%f, %f, %f, %f" % (v0, v1, v2, v3))
		except Exception as E:
			log.info(str(E))
 
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

## Main
# initialize the video stream and allow the camera sensor to  warmup
log.info("initiating video streams ...")
try:
	cam_steering = VideoStream(STEERING_RTP_ADDR).start() #!TODO: Create centralized config for this (SHARED WITH LAUNCH XMLS)
	log.info("started steering video stream ...")
except:
	log.warning("failed to start steering stream ...")
	cam_steering = None

# initialize the video stream and allow the camera sensor to  warmup
try:
	cam_underbody = VideoStream(UNDERBODY_RTP_ADDR).start() #!TODO: Create centralized config for this (SHARED WITH LAUNCH XMLS)
	log.info("started underbody video stream ...")
except:
	log.warning("failed to start underbody stream ...")
	cam_underbody = None
time.sleep(BOOT_DELAY)
log.info("boot complete ...")

if __name__ == '__main__': # check to see if this is the main thread of execution

	# start a thread that will perform stream capture
	t = threading.Thread(target=capture_streams, args=(args["frame_count"],))
	log.info("initiating RTP videostreams thread ...")
	t_streams = threading.Thread(target=capture_streams)
	t_streams.daemon = True
	t_streams.start()

	# start a thread that will perform UDP CAN capture
	log.info("initiating ZMQ J1939 thread ...")
	t_capture_j1939 = threading.Thread(target=capture_j1939)
	t_capture_j1939.daemon = True
	t_capture_j1939.start()

	# start a thread that will perform voltage readings
	log.info("initiating ADIO thread ...")
	t_capture_adio = threading.Thread(target=capture_adio)
	t_capture_adio.daemon = True
	t_capture_adio.start()

	# start the flask app
	log.info("initiating Flask web app ...")
	app.run(host=HOST, port=PORT, debug=DEBUG, threaded=True, use_reloader=False)

## Shutdown
log.info("shutting down ...")

# release the video stream pointer
if cam_steering is not None:
	log.info("releasing steering stream ...")
	cam_steering.stop()

# release the video stream pointer
if cam_underbody is not None:
	log.info("releasing underbody stream ...")
	cam_underbody.stop()

# release the zmq socket
if  is not None:
