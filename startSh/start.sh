#!/bin/bash
cd /home/sz/racecar
export ROS_HOSTNAME=192.168.5.101
export ROS_MASTER_URI=http://192.168.5.101:11311

roslaunch art_racecar Run_car.launch
