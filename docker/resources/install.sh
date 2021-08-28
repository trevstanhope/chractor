#!/bin/bash

sudo apt-get update -y
sudo apt-get install python3 python3-dev python3-pip python3-numpy python3-zmq python3-flask -y
sudo pip3 install imutils

# OpenCV: see https://docs.opencv.org/4.5.2/d2/de6/tutorial_py_setup_in_ubuntu.html
sudo apt-get install cmake
sudo apt-get install gcc g++
sudo apt-get install git
git clone https://github.com/opencv/opencv.git
cd opencv && mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D WITH_GTK=OFF -D WITH_OPENCL=OFF -D BUILD_opencv_gapi=OFF -D BUILD_opencv_python2=OFF -D BUILD_EXAMPLES=OFF -D BUILD_opencv_apps=OFF -D BUILD_DOCS=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_TESTS=OFF ..
make
sudo make install

