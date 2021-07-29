# SLAM-Application
my final year project as a computer and communication engineer.
In this project i created a robot to implement SLAM algorithm. The hardward i used in this project are: raspberry pi 4, arduino uno, raspebrry pi camera module, LIDAR, power bank, battery, 4 DC motors and 2 wheels encoders.
ROS FOXY is the ros version i used, also ubuntu 20.04 is my OS.
to download and intsall ROS Noetic, follow the instructions from the link down below:
http://wiki.ros.org/noetic/Installation/Ubuntu

![complate schematic](https://user-images.githubusercontent.com/73353537/127464745-c2d5dd35-4323-46a6-b5e9-a2720bf9abdd.JPG)

# Project Setup
First of all, you need to clone this repo in the computer and in the raspberry pi. Make sure that your computer and the raspberry pi are in the same network and start pinging to test the connectivity.

After testing testing the connectivity, you can now SSH to the rasberry pi (make sure the SSH is enabled in both raspberry pi and computer). 

Now you need to cd to the ros workspace and export the ROS IP and ROS URI on both the raspberry pi and the computer: 
```
export ROS_IP={ip address}
export ROS_MASTER_URI=http://{ip address}:11311/
```
You can get the ip address of the machine using the command:
```
hostname -I
```
Also don't forget to source your environments:
```
source devel/setup.bash
```
# Testing the LIDAR
Connect the LIDAR to the raspberry pi and run the following command (on the raspberry pi):
```
rosrun delta_2b_lidar delta_2b_lidar_node
```
In the computer open RVIZ (RVIZ is a visualization tool, it comes with ROS if you download the full package) and subscribe to the scan topic.
a small note here, in some cases you need to give permission to the port:
```
sudo chmod 777 /dev/ttyUSB0
```
note that in your computer maybe you use another port (different from ttyUSB0)

if everything is ok you can move to the next step.


# Connect arduino with Raspberry pi
I connected the ardnui with the raspberry pi serially using the rosserial package. first of all make sure that the port have the permission.
then run:
```
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB1 _baud:=115200
```
note: u can create a launch file to launch the lidar and the rosserial at the same time.

# Builing a map using SLAM
I used Hector Mapping to build my map. start the LIDAR node, and one another terminal run the following:
```
roslaunch hector_slam_launch tutorial.launch
```
In RVIZ susbscribe to the map, scan, path and Tf topics. Make sure that the fixed frame is map.
open another terminal and launch the keys controller:
```
roslaunch clawbot_pkg clawbot_driver.launch
 
```
start moving the robot and the map will updated on rviz.

# Save the map
cd to the map folder inside clawbot_package, and run the following:
```
rosrun map_server map_saver -f mymap
```
this line will save 2 file in the map folder. one is .yaml and one .pgm.

# Autonomous navigation
I used navigation stack to perform autonomous navigation. first you need to make sure that move base and acml nodes are installed(usually they will automatically installed with ros).
after saving the map, now we need to run the lidar node, the rosserial node, the robot configuration node and the move base node.
and make sure that in the move base file the name of the name is the same as the name of the map you created.
```
<node name="map_server" pkg="map_server" type="map_server" args="$(find clawbot_pkg)/map/mymap.yaml"/>
```

```
rosrun delta_2b_lidar delta_2b_lidar_node
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB1 _baud:=115200
roslaunch clawbot_pck my_robot_configuration
roslaunch clawbot_pkg movebase
```
Open Rviz and give a goal to the robot from the tools in RVIZ.

# Use the Camera Module
in the raspbeery pi run:
```
roslaunch raspicam_node camerav2_1280x960_10fps.launch enable_raw:=true 
```
in the computer run:
```
rosrun raspicam_node imv_view.py
```
or 
```
rosrun rqt_image_view rqt_image_view
```
