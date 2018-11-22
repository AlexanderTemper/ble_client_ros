# Example Project Spektrum RC with Betaflight and Simblee

Aim of this Project is to control a drone from the computer using a joystick. On Pc side Ros is used to read data from a joystick and send it over BLE to the simblee. The Simblee then generates the Spektrum 2048 frame (only 7 channels are used) which is then sent over the UART to the flight controller.

The Simblee is fail silent which means if no data is received over BLE he don’t output anything to the flight controller. This gives the flight controller the chance to go in failsafe mode.

Components used:

| Component  | |
| ------------- | ------------- |
| Flightcontroller | Matek F405 STD with Betaflight 3.2.2   |
| Bluetooth Modul Drone | Simblee |
| USB Bluetooth Dongle | LogiLink-BT0015 |
| OS | Ubuntu-Mate 18.04 |
| ROS-Version | melodic,1.14.3 |
| Joystick | Sony DualShock 3 |


There a two different implementation for the BLE communication available. The first is based on the Source of the gatttool directly . The Second is based on pygatt which also uses gattool. Both clients implement a reconnect feature if the connection is lost. (the python client reconnects only if he can not sent data, or during first connect)

(Note: currently there is a bug in the first implementation where after a number of data sent no more data are sent.) 

## Setting up gattlib
```
git clone https://github.com/tuw-cpsg/ble_client_ros.git
cd libgatt
$ ./bootstrap
$ ./configure
$ make -j`nproc`
sudo make install
```
## Setting up pygatt
```
sudo apt-get install python-pip
pip install pygatt
```
## Install and Building the ROS Nodes:

1. Create a new folder
```
mkdir -p bleWS/src
cd bleWS
```
2. Clone this repository 
```
git clone https://github.com/ChristianHirsch/ble_client_ros.git
mv ble_client_ros/ src
```
3. Create a new catkin workspace
```
source /opt/ros/melodic/setup.bash
catkin_make
source devel/setup.bash
```
## Using the ROS Nodes:
### Setup Nodes
There are two launch scripts for launching the example.

| script  | |
| ------------- | ------------- |
| [drone.launch](ble_client_node/launch/drone.launch) | launches the cpp node based on gattlib   |
| [python_drone.launch](ble_client_node/launch/python_drone.launch) | launches the python script [droneControl.py](ble_client_node/scripts/droneControl.py)  based on pygatt |

in both files you need too make adjustments to match your controller input device and your Bluetooth Device Address.
```xml
<param name="joy_node/dev"  value="/dev/input/js0" /> 
<param name="droneControl/device" value="C4:D0:0D:79:59:91" />
```
### Launching the example
#### launch the python node
```
roslaunch ble_client_node python_drone.launch --screen
``` 
#### launch the cpp node.
```
roslaunch ble_client_node drone.launch --screen
```

If you get the error:
```
bleWS/devel/lib/ble_client_node/ble_client_node: error while loading shared libraries: libgatt.so.0
```
add the libgatt to the LD_LIBRARY_PATH
```
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```

## Simblee:
Simblee uses the arduino platform. In [Drone.ino](ble_client_ros/spektrumExampleSimbleeSrc/Drone.ino) 
Simblee receives the controll data over BLE and send every 11ms a Spektrum frame over the UART to the Flight Controller. The Flight Controller uses LightTelemetry over the TX Channel (115200 Baud) and send back Telemetry data to the Simblee. Simblee sends this data with BLE back to the ROS Node.
`
## TODO: 
* find bug in cpp node
