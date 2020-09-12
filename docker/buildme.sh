#!/bin/bash
cd resources && rm -f jca_j1939_to_ros.tar &&tar -cvf jca_j1939_to_ros.tar jca_j1939_to_ros
cd -
docker build . -t chractor:latest
