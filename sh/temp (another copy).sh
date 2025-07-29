#!/bin/bash

cd ../object_det/scripts && python3 det.py &
cd /home/nvidia/catkin_ws/src/Challege_ROS/recognize_aruco && python3 image.py & 

#cd ../sensor_pkg && python3 main.py 
sleep 10
gnome-terminal --window --title="faster-lio" -e 'bash -c "roslaunch faster_lio rflysim.launch; exec bash;"'
sleep 10
gnome-terminal --window --title="planner" -e 'bash -c "roslaunch ego_planner rflysim.launch; exec bash;"'
sleep 5
gnome-terminal --window --title="control" -e 'bash -c "roslaunch controller control.launch; exec bash;"'
