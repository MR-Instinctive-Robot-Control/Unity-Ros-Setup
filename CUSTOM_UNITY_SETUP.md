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

# How to start the Robot-HL2 Communication

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

If you want to use the actual robot instead of the gazebo version you have to change the first command slightly:
roslaunch interbotix_xsarm_moveit_interface xsarm_moveit_interface.launch robot_model:=wx250s dof:=6 use_cpp_interface:=true **use_actual:=true**
moveit_interface_gui:=false 