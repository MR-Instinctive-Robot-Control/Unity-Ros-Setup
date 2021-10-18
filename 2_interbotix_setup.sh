echo "Installing librealsense2..."

sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -sc) main" -u
version="2.48.0-0~realsense0.4975"

sudo apt -y install librealsense2-udev-rules=${version}
sudo apt -y install librealsense2-dkms
sudo apt -y install librealsense2=${version}
sudo apt -y install librealsense2-gl=${version}
sudo apt -y install librealsense2-gl-dev=${version}
sudo apt -y install librealsense2-gl-dbg=${version}
sudo apt -y install librealsense2-net=${version}
sudo apt -y install librealsense2-net-dev=${version}
sudo apt -y install librealsense2-net-dbg=${version}
sudo apt -y install librealsense2-utils=${version}
sudo apt -y install librealsense2-dev=${version}
sudo apt -y install librealsense2-dbg=${version}
sudo apt-mark hold librealsense2*
sudo apt -y install ros-melodic-ddynamic-reconfigure

echo "Installing RealSense ROS Wrapper..."
cd ~/catkin_ws/src
git clone git@github.com:IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout 2.3.1

catkin build realsense_ros
source ~/catkin_ws/devel/setup.bash

echo "Installing Apriltag ROS Wrapper..."
cd ~/catkin_ws/src
git clone https://github.com/AprilRobotics/apriltag.git
git clone https://github.com/AprilRobotics/apriltag_ros.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

catkin build 
source ~/catkin_ws/devel/setup.bash

echo "Installing ROS packages for the Interbotix Arm..."
cd ~/catkin_ws/src
git clone https://github.com/Interbotix/interbotix_ros_core.git
git clone https://github.com/Interbotix/interbotix_ros_manipulators.git
git clone https://github.com/Interbotix/interbotix_ros_toolboxes.git
cd interbotix_ros_manipulators && git checkout melodic && cd ..
rm interbotix_ros_core/interbotix_ros_xseries/CATKIN_IGNORE
rm interbotix_ros_manipulators/interbotix_ros_xsarms/CATKIN_IGNORE
rm interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_perception/CATKIN_IGNORE
rm interbotix_ros_toolboxes/interbotix_perception_toolbox/CATKIN_IGNORE
rm interbotix_ros_toolboxes/interbotix_xs_toolbox/CATKIN_IGNORE
rm interbotix_ros_toolboxes/interbotix_common_toolbox/interbotix_moveit_interface/CATKIN_IGNORE
cd interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk
sudo cp 99-interbotix-udev.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

catkin build -c
source ~/catkin_ws/devel/setup.bash