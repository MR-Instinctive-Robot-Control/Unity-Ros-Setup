#!/bin/bash

cd ~/catkin_ws/src

git clone git@github.com:Unity-Technologies/ROS-TCP-Endpoint.git

catkin build 
source ~/catkin_ws/devel/setup.bash
