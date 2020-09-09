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

To get OpenCV:

	sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
	sudo apt-get install python3.5-dev
	git clone https://github.com/Itseez/opencv.git
	mkdir opencv/build
	cd opencv/build
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
	make -j $(nproc --all)
	make install


To build ROS workspace:

	catkin_make

To build the JS applet, from the src/app/js/ directory:
    
    npm install -g typescript
    tsc

To get Python:

	pip install -r requirements.txt

## Overview
At boot, the device will initialize an AP-mode wireless network. Afterwards, bootloader.py will be executed as a daemon process. If it detects a USB drive, it will automatically copy files from /path/to/media/app (Need to incorporate safety-check).

Next, the (most recent) the app.py will be executed as a service. This service will attempt to establish communication with the controller and any CAN/ethernet devices (e.g. camera), and launch microwebserver for the mobile application. 

The controller is equipped with a CAN transceiver, and can be set-up for a wide variety of I/O 

## Parts
All critical components required for the device are listed below (NOTE: some minor parts may be omitted):

### Electrical Hardware
* 12VDC~40VDC to 5VDC Waterproof Voltage Convert Transformer
* Littlefuse In-line fuse holder
* DT-P 4pin
* DT-S 12pin A-keyed
* DT-S 12pin B-keyed
* DT-S 12pin D-keyed
* DT-S 12pin D-keyed
* DTM-S 12pin B-keyed
* Cinch ModICE LE
	* http://www.peigenesis.com/en/about-us/5540:partner-with-distributors-for-custom-connector-designs.html

### Sensors
* DS18B20 Temperature Sensor
* Dorman #924-261 Suspension Sensor
* Adafruit 10-DOF IMU (LSM303DLHC, L3GD20, and BMP180)
	* https://www.adafruit.com/product/1604
	* https://github.com/adafruit/Adafruit_10DOF

### Connectors
Sensor Harness, Bulgin 400 Series (8 position)

* Female Receptacle (Panel-Mount) - https://www.digikey.com/product-detail/en/bulgin/PX0413-08S-PC/708-1083-ND/1625848
* Male Plug (In-Line) - https://www.digikey.com/product-detail/en/bulgin/PX0410-08P-6065/708-1034-ND/1625799
* Male Pins (22-26 AWG) - https://www.digikey.com/product-detail/en/bulgin/SA3348/708-1089-ND/1625854
* Female Sockets (22-26 AWG) - https://www.digikey.com/product-detail/en/bulgin/SA3347-1/708-1093-ND/1625858

Power Supply, Bulgin 400 Series (2 position)

* Male Receptacle (Panel-Mount) - https://www.digikey.com/product-detail/en/bulgin/PX0413-02P-PC/708-1072-ND/1625837
* Female Plug (In-Line) - https://www.digikey.com/product-detail/en/bulgin/PX0410-02S-5560/708-1037-ND/1625802
* Male Pins (20-24 AWG) - https://www.digikey.com/product-detail/en/bulgin/SA3350-1/708-1092-ND/1625857
* Female Sockets (20-24 AWG) - https://www.digikey.com/product-detail/en/bulgin/SA3349-1/708-1091-ND/1625856

Camera Adaptor, Bulgin 400 Series (4 position)

* Male Plug (In-Line) - https://www.digikey.com/product-detail/en/bulgin/PX0410-04P-4550/708-1032-ND/1625797
* Female Receptacle (In-Line) - https://www.digikey.com/product-detail/en/bulgin/PX0411-04S-4550/708-1053-ND/1625818
* Male Pins (22-26 AWG) - https://www.digikey.com/product-detail/en/bulgin/SA3348/708-1089-ND/1625854
* Female Sockets (22-26 AWG) - https://www.digikey.com/product-detail/en/bulgin/SA3347-1/708-1093-ND/1625858

Camera Input, Bulgin Data Series (USB Mini)

* Extension Cable - https://www.digikey.com/product-detail/en/bulgin/PX0441-4M50/708-1231-ND/1625996
* Female Receptacle (Panel-Mount) - https://www.digikey.com/product-detail/en/bulgin/PX0443/708-1235-ND/1626000

Switched Power Out, TE Connectivity (4 position)

* Female Receptacle (Panel-Mount) https://www.digikey.com/product-detail/en/te-connectivity-amp-connectors/206430-1/A1360-ND/19367
* Male Plug (In-Line) - https://www.digikey.com/product-detail/en/te-connectivity-amp-connectors/206429-1/A1357-ND/19358
* Male Pins (20-24 AWG) - https://www.digikey.com/product-detail/en/te-connectivity-amp-connectors/2-66102-5/A31989TR-ND/808381

Firewall Port, Amphenol Industrial Operations (18.0mm ~ 25.0mm, M32x1.5)

* Strain Relief - http://www.digikey.com/scripts/DkSearch/dksus.dll?Detail&itemSeq=214306199&uq=636173608808318739
