#!/bin/bash

# roscore &
# sleep 5

cd /root/catkin_ws/src/Challege_ROS/sensor_pkg && python3 main.py &
使用 sleep 命令等待1.1秒
sleep 20

# 在新终端窗口中运行第一个命令
xterm -hold -e "cd /root/catkin_ws/src/Challege_ROS/object_det/scripts && python3 det.py" &

# 等待1秒
sleep 1

# 在新终端窗口中运行第二个命令
xterm -hold -e "cd /root/catkin_ws/src/Challege_ROS/recognize_aruco && python3 image.py" &

# 等待4秒
sleep 5

# 在新终端窗口中运行第三个命令
xterm -hold -e "roslaunch faster_lio rflysim.launch" &

# 等待3秒
sleep 5

# 在新终端窗口中运行第四个命令
xterm -hold -e "roslaunch ego_planner rflysim.launch" &

# 等待10秒
sleep 6

# 在新终端窗口中运行第五个命令
xterm -hold -e "roslaunch controller sim.launch"


# cd /home/catkin_ws/src/Challege_ROS/sensor_pkg && python3 main.py &
# sleep 1 && cd /home/catkin_ws/src/Challege_ROS/object_det/scripts && python3 det.py &
# sleep 2 && cd /home/catkin_ws/src/Challege_ROS/recognize_aruco && python3 image.py &
# sleep 5 && roslaunch faster_lio rflysim.launch &
# sleep 8 && roslaunch ego_planner rflysim.launch &
# sleep 11 && roslaunch controller control.launch



# sleep 1.5 
# roslaunch ego_planner rviz.launch
