#!/bin/bash

BASEDIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd
# Ros noetic installation and catkin setup
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update && sudo apt install ros-melodic-desktop-full -y
sudo apt install ros-melodic-moveit \
	ros-melodic-rosbridge-suite \
	ros-melodic-ros-control \
	ros-melodic-ros-controllers \
	ros-melodic-tf2-web-republisher -y

source /opt/ros/melodic/setup.bash

sudo apt install python-rosdep \
	python-rosinstall \
	python-rosinstall-generator \
	python-wstool \
	build-essential \
	python-rosdep -y

sudo rosdep init && rosdep update

sudo apt install python-catkin-tools \
	python-catkin-lint \
	python-pip -y

pip install osrf-pycommon

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin init
catkin config --merge-devel
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

cd ~/catkin_ws/src
git clone git@github.com:catkin/catkin_simple.git
git clone git@github.com:ethz-asl/eigen_catkin.git

catkin build -c
source ~/catkin_ws/devel/setup.bash

if ! grep -Fxq "source ~/catkin_ws/devel/setup.bash" ~/.bashrc; then
	# add to beginning of file
	echo "source ~/catkin_ws/devel/setup.bash" | cat - ~/.bashrc > temp && mv temp ~/.bashrc
fi
if ! grep -Fxq "source /opt/ros/melodic/setup.bash" ~/.bashrc; then
	# add to beginning of file (before catkin workspace sourcing!)
	echo "source /opt/ros/melodic/setup.bash" | cat - ~/.bashrc > temp && mv temp ~/.bashrc
fi
if ! grep -Fxq "export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0.0" ~/.bashrc; then
	# for display forwarding though X11, add to end of file 
    echo "export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0.0" >> ~/.bashrc
fi

xargs -d '\n' -- sudo apt install -y < $BASEDIR/base_packages.txt

sudo -H pip install --upgrade pip
sudo -H pip install -r $BASEDIR/requirements.txt

wstool init

# Setup Environment Variables
if [ -z "$ROS_IP" ]; then
  echo "Setting up Environment Variables..."
  echo 'export ROS_IP=$(echo `hostname -I | cut -d" " -f1`)' >> ~/.bashrc
  echo -e 'if [ -z "$ROS_IP" ]; then\n\texport ROS_IP=127.0.0.1\nfi' >> ~/.bashrc
else
  echo "Environment variables already set!"
fi
