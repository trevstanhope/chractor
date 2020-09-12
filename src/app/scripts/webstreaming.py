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

# 
DEBUG = False
VIDEO_HEIGHT = 240
VIDEO_WIDTH = 320
HOST = "0.0.0.0"
PORT = 8000
FRAMERATE = 5
BOOT_DELAY = 2.0

# 
outputFrame_steering = np.zeros((VIDEO_HEIGHT,VIDEO_WIDTH,3), np.uint8)
outputFrame_undercarriage = np.zeros((VIDEO_HEIGHT,VIDEO_WIDTH,3), np.uint8)
lock = threading.Lock()
app = Flask(__name__) # initialize a flask object

# initialize the video stream and allow the camera sensor to  warmup
try:
	vs_steering = VideoStream("rtp://192.168.40.181:50008").start()
	print("Started steering stream")
except:
	print("Failed to start steering stream")
	vs_steering = None

# initialize the video stream and allow the camera sensor to  warmup
try:
	vs_undercarriage = VideoStream("rtp://192.168.40.91:50004").start()
	print("Started undercarriage stream")
except:
	print("Failed to start undercarriage stream")
	vs_undercarriage = None
time.sleep(BOOT_DELAY)

@app.route("/")
def index():

	# return the rendered template
	return render_template("index.html")

def capture_streams():
	# grab global references to the video stream(s), output frame, and
	# lock variables
	global vs_steering, vs_undercarriage, outputFrame_steering, outputFrame_undercarriage, lock

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
		if vs_undercarriage is not None: 
			frame_undercarriage = vs_undercarriage.read()
			if frame_undercarriage is not None:
				frame_undercarriage = imutils.resize(frame_undercarriage, width=VIDEO_WIDTH, height=VIDEO_HEIGHT)
				with lock:
					outputFrame_undercarriage = frame_undercarriage.copy()
			else:
				 outputFrame_undercarriage = np.zeros((VIDEO_HEIGHT,VIDEO_WIDTH,3), np.uint8)

def generate():
	# grab global references to the output frame and lock variables
	global outputFrame_steering, outputFrame_undercarriage, lock
	# loop over frames from the output stream
	while True:
		# wait until the lock is acquired
		with lock:
			# check if the output frame is available, otherwise skip
			# the iteration of the loop
			outputFrame_combo = np.hstack((outputFrame_steering, outputFrame_undercarriage))
			# encode the frame in JPEG format
			(flag, encodedImage_combo) = cv2.imencode(".jpg", outputFrame_combo)
			# ensure the frame was successfully encoded
			if not flag:
				continue
		
		# yield the output frame in the byte format
		yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage_combo) + b'\r\n')

@app.route("/video_feed")
def video_feed():
	# return the response generated along with the specific media
	# type (mime type)
	return Response(generate(), mimetype = "multipart/x-mixed-replace; boundary=frame")

# check to see if this is the main thread of execution
if __name__ == '__main__':
	# construct the argument parser and parse command line arguments
	#ap = argparse.ArgumentParser()
	#ap.add_argument("-i", "--ip", type=str, required=True, help="ip address of the device")
	#ap.add_argument("-o", "--port", type=int, required=True, help="ephemeral port number of the server (1024 to 65535)")
	#ap.add_argument("-f", "--frame-count", type=int, default=32, help="# of frames used to construct the background model")
	#args = vars(ap.parse_args())
	
	# start a thread that will perform motion detection
	#t = threading.Thread(target=capture_streams, args=(args["frame_count"],))
	t = threading.Thread(target=capture_streams)
	t.daemon = True
	t.start()
	# start the flask app
	#app.run(host=args["ip"], port=args["port"], debug=DEBUG, threaded=True, use_reloader=False)
	app.run(host=HOST, port=PORT, debug=DEBUG, threaded=True, use_reloader=False)

# release the video stream pointer
if vs_steering is not None:
	vs_steering.stop()

# release the video stream pointer
if vs_undercarriage is not None:
	vs_undercarriage.stop()
