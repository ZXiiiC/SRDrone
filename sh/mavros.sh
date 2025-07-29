#!/bin/bash

roslaunch mavros px4.launch fcu_url:="udp://:20101@192.168.2.3:20100" &

sleep 3
gnome-terminal --tab --title="drone state" -e 'bash -c "rostopic echo /mavros/state; exec bash"'  
gnome-terminal --tab --title="drone pose" -e 'bash -c "rostopic echo /mavros/local_position/pose; exec bash"'  
gnome-terminal --tab --title="target pose" -e 'bash -c "rostopic echo /mavros/setpoint_raw/local; exec bash"'  
gnome-terminal --tab --title="lio_output" -e 'bash -c "rostopic echo /mavros/odometry/out; exec bash"'  
