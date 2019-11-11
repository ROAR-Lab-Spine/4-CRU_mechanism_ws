#!/bin/bash

echo "Start ROS-Arduino Serial Communication..."

# Setting permission to access Arduino Mega 2560 port /dev/ttyACM0 
# sudo usermod -a -G dialout $USER
# sudo chmod a+rw /dev/ttyACM0 

roslaunch arduino_low_lvl_ctrl arduino_low_lvl_ctrl.launch &

roslaunch kinematic_model kinematic_model.launch &

rqt 

# rqt --standalone rqt_robot_ctrl_gui


# Show the velocity commands in xterm 
# xterm -hold -e  "rostopic echo /cmd_vel"


# Send the SIGKILL (9) signal to all processes
killall -9 rosmaster 
killall python