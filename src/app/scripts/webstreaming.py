# import the necessary packages
from lib.singlemotiondetector import SingleMotionDetector
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

debug = True
outputFrame = None
lock = threading.Lock()
app = Flask(__name__) # initialize a flask object
# initialize the video stream and allow the camera sensor to  warmup

try:
	if debug: vs_steering = VideoStream(0).start()
	else: vs_steering = VideoStream("rtp://192.168.40.181:50008").start()
	print("Started steering stream")
except:
	print("Failed to start steering stream")
	vs_steering = None

try:

	if debug: vs_undercarriage = vs_steering
	else: vs_undercarriage = VideoStream("rtp://192.168.40.40:50004").start()
	print("Started undercarriage stream")
except:
	print("Failed to start undercarriage stream")
	vs_undercarriage = None
time.sleep(2.0)

@app.route("/")
def index():

	# return the rendered template
	return render_template("index.html")

def detect_motion(frameCount):
	# grab global references to the video stream(s), output frame, and
	# lock variables
	global vs_steering, vs_undercarriage, outputFrame_steering, outputFrame_undercarriage, lock

	# loop over frames from the video stream
	while True:
		# read the next frame from the video stream, resize it,
		# convert the frame to grayscale, and blur it
		if vs_steering is not None: 
			frame_steering = vs_steering.read()
			frame_steering = imutils.resize(frame_steering, width=400)
			#cv2.putText(frame_steering, "Steering", (10, frame_steering.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
			
			# acquire the lock, set the output frame, and release the
			# lock
			with lock:
				outputFrame_steering = frame_steering.copy()

		if vs_undercarriage is not None: 
			frame_undercarriage = vs_undercarriage.read()
			frame_undercarriage = imutils.resize(frame_undercarriage, width=400)
			#cv2.putText(frame_undercarriage, "Undercarriage", (10, frame_undercarriage.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

			# acquire the lock, set the output frame, and release the
			# lock
			with lock:
				outputFrame_undercarriage = frame_undercarriage.copy()

def generate():
	# grab global references to the output frame and lock variables
	global outputFrame_steering, outputFrame_undercarriage, lock
	# loop over frames from the output stream
	while True:
		# wait until the lock is acquired
		with lock:
			# check if the output frame is available, otherwise skip
			# the iteration of the loop
			if outputFrame_steering is None:
				continue
			if outputFrame_undercarriage is None:
				continue

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
	ap = argparse.ArgumentParser()
	ap.add_argument("-i", "--ip", type=str, required=True, help="ip address of the device")
	ap.add_argument("-o", "--port", type=int, required=True, help="ephemeral port number of the server (1024 to 65535)")
	ap.add_argument("-f", "--frame-count", type=int, default=32, help="# of frames used to construct the background model")
	args = vars(ap.parse_args())
	# start a thread that will perform motion detection
	t = threading.Thread(target=detect_motion, args=(args["frame_count"],))
	t.daemon = True
	t.start()
	# start the flask app
	app.run(host=args["ip"], port=args["port"], debug=True, threaded=True, use_reloader=False)

# release the video stream pointer
if vs_steering is not None:
	vs_steering.stop()
