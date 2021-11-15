# Cube tracking setup on ROS for HL2 visualization

Step by step guide to run a cube pose estimation ROS node via a RealSense depth camera D455.

## ROS setup

### importing the node in the catkin workspace
- Download the *cube_tracking.zip* from here: https://gitlab.ethz.ch/mr-instinctive-robot/aruco-markers-and-unity-holograms/-/tree/ivan_Holograms_creation

- Unzip the folder and copy it in /home/username/catkin_ws/src/

- Run the following command to build the package:
```shell
catkin build cube_tracking
```


### Install RealSense dependencies
- Follow the installation steps at the following link (under the section **Installing the packages**): https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

- Once followed the previous steps, by running
'''shell
realsense-viewer
'''
a window should appear displaying the RGB and depth camera streams.

## Installing requirements for the OpenCV module
- Install the following packages:
```shell
pip install opencv-python==4.2.0.32
```
(that is the compatible version with python2)

```shell
pip install opencv-contrib-python
```


```shell
pip install scipy
```


# Running the  ROS node
To run the ROS cube tracking node, run the following commands.
Terminal 1:
```shell
roscore
```
Terminal 2:

```shell
rosrun cube_tracking publisher_cube_position.py 
```

Terminal 3 (to display the cube pose and the aruco marker id):
```shell
rostopic echo /publisher_cube_position/cube_pose
```
```shell
rostopic echo /publisher_cube_position/marker_id
```


After changing the ROS IP address (echo $ROS_IP) on Unity and building the project, you should also run the following to instantiate the TCP connection:

```shell
roslaunch ros_tcp_endpoint endpoint.launch
```

