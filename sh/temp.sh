#!/bin/bash

# roscore &
# sleep 5

cd /home/catkin_ws/src/Challege_ROS/sensor_pkg && python3 main.py &
sleep 20
cd /home/catkin_ws/src/Challege_ROS/object_det/scripts && python3 det.py &
sleep 1
cd /home/catkin_ws/src/Challege_ROS/recognize_aruco && python3 image.py &
sleep 1
sleep 4 
roslaunch faster_lio rflysim.launch &
sleep 3 
roslaunch ego_planner rflysim.launch &
sleep 10 
roslaunch controller control.launch

# cd /home/catkin_ws/src/Challege_ROS/sensor_pkg && python3 main.py &
# sleep 1 && cd /home/catkin_ws/src/Challege_ROS/object_det/scripts && python3 det.py &
# sleep 2 && cd /home/catkin_ws/src/Challege_ROS/recognize_aruco && python3 image.py &
# sleep 5 && roslaunch faster_lio rflysim.launch &
# sleep 8 && roslaunch ego_planner rflysim.launch &
# sleep 11 && roslaunch controller control.launch



# sleep 1.5 
# roslaunch ego_planner rviz.launch
