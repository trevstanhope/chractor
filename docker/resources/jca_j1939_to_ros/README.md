# J1939 to ROS converter

This is the ROS package that contains the generic J1939 to ros converter for the hummingbird. It gets the J1939 messages from the J1939 stack running on the Hummingbird. Right now this uses ZMQ but this may change in the future. 

### Prerequisites

This package is intended to be used with an already existing ROS workspace, and should be pulled in as a submodule in a ROS workspace

### Configuration

This package reads in the configuration from the jca_j1939_to_ros.yaml file.
The parameters in the yaml file are as follows:

#### J1939_ZMQ_SOCKET
This parameter sets the address of the ZMQ publisher that the ros node subscribes to, it should be in the format of "J1939_ZMQ_SOCKET: tcp://{J1939_ZMQ_STACK_IP}:{J1939_ZMQ_STACK_PORT}". 
For example: "J1939_ZMQ_SOCKET: tcp://192.168.167.54:5678"

#### J1939_NODE
This is the name for the ros node. The node is set up as not anonymous so this must be a unique name in the ros network.
For example: "J1939_NODE: HB_J1939_can0_raw"

#### J1939_TOPIC
This is the name for the ros topic. This can be any unique or existing topic that uses the j1939 message structure found in msg/j1939.msg
For example: "J1939_TOPIC: HB_J1939_can0_raw"

#### J1939_PGN_LIST_STR
This is a yaml list of PGNs that the node will rebroadcast. 
If this list is left blank, or removed from the yaml file, the node will rebroadcast all PGNs (0000-FFFF).
To specify which PGNs to rebroadcast, simply add it to the yaml list.
The list does not need to be in order. 
For example:

````
J1939_PGN_LIST_STR:
    -F004
    -F003
````
This list will rebroadcast the EEC1 and EEC2 PGNs.




