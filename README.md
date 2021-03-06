# Chractor
Embedded system for Cherokee XJ offroad management
Install Debian Linux 8.3.0 (headless) to the device.
Some features which are possible to implement include:
- lockable differentials
- CAN monitoring 
- roll/pitch
- axle articulation
- additional temp sensors
- e-fan override
- light control
- cameras

## Installation (Auto)

Get the repository:

    su
    cd /root
    git clone https://github.com/trevstanhope/chractor

Run the install script:

    cd chractor
    ./install.sh

# Installation (Manual)
Get the repository:

    su
    cd /root
    git clone https://github.com/trevstanhope/chractor
    cd chractor

To get OpenCV (from source):
	sudo apt-get update
	sudo apt-get upgrade
	sudo apt-get install python3-numpy
	sudo apt-get install build-essential cmake git pkg-config
	sudo apt-get install libjpeg8-dev libtiff4-dev libjasper-dev libpng12-dev
	sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
	sudo apt-get install libatlas-base-dev gfortran
	sudo apt-get install python3.5-dev
	git clone https://github.com/opencv/opencv.git
	git checkout 3.0.0
	mkdir opencv/build
	cd opencv/build
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
    cmake -D CMAKE_BUILD_TYPE=Release -D WITH_QT=OFF -D WITH_GTK=OFF -D BUILD_NEW_PYTHON_SUPPORT=ON -D BUILD_opencv_python3=ON -D HAVE_opencv_python3=ON -D PYTHON_DEFAULT_EXECUTABLE=/usr/bin/python3.5 ..
	make -j $(nproc --all)
	make install


To build ROS workspace:

	catkin_make

If using the NodeJS application, in order to build the JS applet, from the src/app/js/ directory:
    
    npm install -g typescript
    tsc

To get Python:

	sudo apt-get install python3-pip
	pip install numpy
	pip install opencv-contrib-python
	pip install imutils
	pip install -r requirements.txt

## Overview
At boot, the device will initialize an AP-mode wireless network. Afterwards, bootloader.py will be executed as a daemon process. If it detects a USB drive, it will automatically copy files from /path/to/media/app (Need to incorporate safety-check).

Next, the (most recent) the app.py will be executed as a service. This service will attempt to establish communication with the controller and any CAN/ethernet devices (e.g. camera), and launch microwebserver for the mobile application. 

The controller is equipped with a CAN transceiver, and can be set-up for a wide variety of I/O 

Web UI is located at 172.24.1.1 if connected via WiFi, or 192.168.40.2 if over ethernet

If it is necessary to update the device, bridge WLAN over ethernet (see the config in /etc/network/interfaces.d/eth0.cfg.bridge) 

## Docker
Docker files are located in docker/

These consist of the Dockerfile, and resources that are copied to the container

To rebuild the container, run buildme.sh

## WebUI
The web application is a simple interface based on Flask

	python3 webstreaming.py


## Video System
Recommended settings in order to run two (2) RTP video streams:

* 640 x 480 MJPEG
* <5 Mb/s
* 30 frames/s
* 192.168.40.x

## Network Info
* ECU is 192.168.40.2
* Diagnostics laptop UI is 192.168.40.3
* Gateway is 192.168.40.1
* Netmask is 255.255.255.0

## Parts
For details see PARTS.md
