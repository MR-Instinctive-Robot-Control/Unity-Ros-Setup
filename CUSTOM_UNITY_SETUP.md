# Mixed Reality Group Only
- If you do not have git installed, install git for Windows first (https://git-scm.com/download/win), ```restart``` after installing git

- Open Unity hub - if you have been added to the project via collaborate (ask jonny to add you)(sign in to collaborate here: https://dashboard.unity3d.com) - you should see the Human Robot Manipulator in you projects. If you open it it will download all the Assets and Packages for the Project. 

- In the project folders navigate to Packages/Robotics Visualization/Runtime and double click the ```.asmdef``` file (blue puzzle icon). An editor should open and modify the included platform section to the following:

```
"includePlatforms": [
        "Editor",
        "LinuxStandalone64",
        "macOSStandalone",
        "WSA",
        "WindowsStandalone32",
        "WindowsStandalone64"
    ],
```

- Navigate to Packages/URDF Importer/Runtime and do the same for the ```.asmdef``` there

- Open the scene from Assets/Scenes and double click on MainScene to open it. 

- If you are using a trajectory controller, we found that using the default kinematics solver often resulted in no feasible trajectories being found. This was solved by changing the kinematics solver, to do so, follow these steps:

    1. ```sudo apt install ros-melodic-trac-ik-kinematics-plugin```
    2. Navigate to and open the file: in ```~\catkin_ws\src\interbotix_ros_manipulators\interbotix_ros_xsarms\interbotix_xsarm_moveit\config\kinematics.yaml```
    3. Change the content to the following

```xml
# position-only-ik and orientation-only-ik don't work unless position_only_ik
# is set to true for the KDL and trac_ik plugins

# interbotix_arm:
#   kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
#   kinematics_solver_search_resolution: 0.005
#   kinematics_solver_timeout: 0.005
#   position_only_ik: true

interbotix_arm:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_timeout: 0.005
  solve_type: Speed
  position_only_ik: true

# Position-only-ik and orientation-only-ik work automatically for the LMA plugin

# interbotix_arm:
#   kinematics_solver: lma_kinematics_plugin/LMAKinematicsPlugin
#   kinematics_solver_search_resolution: 0.005
#   kinematics_solver_timeout: 0.005
```

- For using our custom smooth hand following controller you need to install scipy in ubuntu:
```shell 
pip install scipy
```

# How to start the Robot-HL2 Communication with a trajectory controller

Open up two terminals in your VMware Ubuntu 18.04 virtual machine:
1. Terminal:  
    roslaunch interbotix_xsarm_moveit_interface xsarm_moveit_interface.launch robot_model:=wx250s dof:=6 use_cpp_interface:=true **use_gazebo:=true**
    moveit_interface_gui:=false  
    Or if you want to use the actual robot:  
    roslaunch interbotix_xsarm_moveit_interface xsarm_moveit_interface.launch robot_model:=wx250s dof:=6 use_cpp_interface:=true **use_actual:=true**
moveit_interface_gui:=false
2. Terminal:  
    rosservice call /gazebo/unpause_physics  
    Then  
    roslaunch ros_tcp_endpoint endpoint.launch

# How to start smooth position controller

1. Clone the MR Instincive Robot Control Repo into your ubuntu catkin_ws:
```git clone git@gitlab.ethz.ch:mr-instinctive-robot/mr-instinctive-robot-control.git```

2. Build it and source it
``catkin build interbotix_hand_joy && source ~/catkin_ws/devel/setup.bash```

3. In two terminals launch (use sim:=true if not on real robot, otherwise default is sim=false):
    - ```roslaunch interbotix_hand_joy hand_joy.launch robot_model:=wx250s use_sim:=true```
    - ```roslaunch ros_tcp_endpoint endpoint.launch```