# unity_ros_setup

Step by step guide on getting a ros environment on your windows machine, which is connected to unity and the hololens 2.

## linux and ros setup

- Install wsl by opening the cmd promtp (just search for cmd) and run:
    wsl --install -d Ubuntu-18.04

- Install *Windows Terminal* from the microsoft app store.

- When opening the windows terminal the first time, click on the arrow pointing down in the tab area and go to settings. Select Ubuntu-18.04 as your standard profile and then in the ubuntu specific settings select your starting folder to be: 
    \\wsl$\Ubuntu-18.04\home\jonathan

- Restart Windows terminal and you are greeted with (the familiar) ubuntu terminal. 

- Get some basic dependencies:

```shell
sudo apt update
sudo apt install build-essential git xclip
```

Now is a good time to set up your git. First enter your information. Replace <Your_name> and <Your_Email> with your credentials.

```shell
git config --global user.name "<Your_name>"
git config --global user.email "<Your_Email>"
```

Then create a ssh keypair with <Your_Email> to establish a connection to github and gitlab.

```shell
ssh-keygen -t ed25519 -C "<Your_Email>"
```

Accept the default location and enter an optional passphrase that you will have to enter everytime you authorize a new ssh connection.

Now copy the public key:

```shell
xclip -sel clip < ~/.ssh/id_ed25519.pub
```

Go to https://gitlab.ethz.ch/-/profile/keys and paste the copied key into the Key text box, the title corresponds to the computer that you are using - then press Add key.

Go to https://github.com/settings/keys and add your key there too.

Now you can get the setup scripts from this repo by cloning them to your home folder (or wherever you prefer):
```shell
cd
git clone git@gitlab.ethz.ch:mr-instinctive-robot/unity_ros_setup.git
```
Now run the install script that will download ROS Melodic and set up your system and catkin workspace for you:

```shell
cd unity_ros_setup
./1_base_setup.sh
```
Back in windows, we need to run an X-Server to be able to display graphical output from ubuntu programs. For this download from here:
https://sourceforge.net/projects/vcxsrv/

Then refer to this tutorial for the settings https://jack-kawell.com/2020/06/12/ros-wsl2/.
Alternatively just create a shortcut on your desktop the the config.xlaunch file located in this package, starting XLaunch via this will automatically set it up correctly. Don't forget to start XLaunch, otherwise graphical Ubuntu programs might hang up.

- At this point you can test if everything worked correctly by opening a new terminal in wsl (should source your ros workspace) and typing rviz which should bring up a graphical ros program.

# Interbotix Setup

Install the required packages to simulate and control the robot arm

```shell
cd unity_ros_setup
./2_interbotix_setup.sh
```

# Unity Setup

First follow the general instructions in the pdf found in the ressources folder. 

Then download the following two repos (in windows, I just did it as a zip folder)
https://github.com/Unity-Technologies/URDF-Importer
https://github.com/Unity-Technologies/ROS-TCP-Connector

Extract them if you downloaded them as a zip folder.
Then open your unity project. 
Click on Window -> Package Manager. Click + in upper corner. Add from disk. Navigate to the extracted zip folder
In URDF Import find the com.unity.robotics.urdf-importer folder and select the package.json file within. Click on import. 

Then do the same thing for the ROS-TCP-CONNECTOR where you need to find the com.unity.robotics.ros-tcp-connector folder and select the package.json file within. Again click on import.

# Setting up the robot

- In the Asset Panel, create a new folder called URDF. From Linux, copy the interbotix_xsarm_descriptions folder into this newly created folder.

- Also in Linux, navigate to : `cd ~/catkin_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_descriptions/urdf`
and run this to convert the xacro to urdf:

```shell
rosrun xacro xacro --inorder -o /home/$USER/wx250s.urdf ./wx250s.urdf.xacro
```

- Then copy the `wx250s.urdf` which was just created in your user home directory to the Assets/URDF folder in Unity as well.

- In Unity right click the newly imported urdf file and click `Import Robot from selected URDF File`. Accept the default settings and click import.

- Open the Physics Project Settings (in the top menu bar, Edit > Project Settings > Physics) and ensure the Solver Type is set to Temporal Gauss Seidel. This prevents erratic behavior in the joints that may be caused by the default solver.

> **_Note_** : The world-space origin of the robot is defined in its URDF file (origin of the base link).

> **_Note_** : Going from Unity world space to ROS world space requires a conversion. Unity's (x,y,z) is equivalent to the ROS (z,-x,y) coordinate.

- Select the imported wx250s in the scene hirarchy and in the Inpsector scoll to the *Controller (Script)* portion. Set the Stiffness to 10'000, the Damping to 100, the Force Limit to 1'000 the Speed to 30 and the Acceleration to 10. These setting allow the robot to stay in position without the joints slipping.

 ![controller setting](/img/1_controller.png)

- In the Hierarchy Window click the arrow to the left of wx250s to expand the GameObject tree, down to wx250s/base_link. Toggle on `Immovable` for the base_link under Articulation Body.

- Now pressing play allows us to use the controller intergrated in the URDF Importer. After pressing play, the arrow keys allow us to select the joints and move them. 

# ROS-Unity Integration
- If everything is setup correctly, you should have a `Robotics Tab` in the top bar. In the submenu select `Generate Ros Messages`. Navigate to your ubuntu catkin workspace. Select and build the messages that you would like to use.  Alternatively you can also simply drag your ros module folder into the Unity Assets/RosMessages Folder and the message generator will automatically detect messages and build them.

- For generic ros messages like geometry_msgs don't need to be imported and can be directly used, as can be seen in the example in the next section

# Writing Custom Scripts

In the Asset Folder create a new Folder called Scripts. As an example lets create a simple message publisher. Right click in the Scripts Folder and select `Create` -> `C# Script` and call it *generic_message_pub*. Now double click the script, which should open visual studio, replace the content with the following:

```csharp
using System;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using UnityEngine;

public class GenericPublisher : MonoBehaviour
{
    public static readonly string LinkName = "wx250s/wrist_link";

    // Variables required for ROS communication
    [SerializeField]
    string m_TopicName = "/unity/wrist_position";

    [SerializeField]
    GameObject m_Robot;

    // Wrist Joint
    // UrdfJointRevolute m_JointArticulationBody;

    // ROS Connector
    ROSConnection m_Ros;

    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<PoseMsg>(m_TopicName);

        // m_JointArticulationBody = m_Robot.transform.Find(LinkName).GetComponent<Transform>();
    }

    public void Publish()
    {
        // create the message
        var msg = new PoseMsg
        {
            position =  m_Robot.transform.position.To<FLU>(),
            orientation = Quaternion.Euler(90, m_Robot.transform.eulerAngles.y, 0).To<FLU>()
        };

        // Finally send the message to server_endpoint.py running in ROS
        m_Ros.Publish(m_TopicName, msg);
    }
}
```

- Return to the Unity Editor. Now with the publisher script added, it needs to be added to the Unity world to run its functionality.

- Right click in the Hierarchy window and select "Create Empty" to add a new empty GameObject. Name it Publisher. Add the newly created GenericPublisher component to the Publisher GameObject by selecting the Publisher object. Click "Add Component" in the Inspector, and begin typing "GenericPublisher." Select the component when it appears.

- Note that this component shows empty member variables in the Inspector window, which need to be assigned. 
In the Robot field select the object for which you want to publish the pose - in this example we will be publishing the pose of wx250s/wrist_link.

# TCP Connection

[//]: # (Make sure that the ROS_IP is passed to moveit for the TCP connection., this is a comment)

- Next, the ROS TCP connection needs to be created. Select ```Robotics -> ROS Settings``` from the top menu bar.

- In the ROS Settings window, the ```ROS IP Address``` should be the IP address of your ROS machine (not the one running Unity).

    - Find the IP address of your ROS machine. In Ubuntu, open a terminal window, and enter ```hostname -I```. If you followed the setup instructions you can also just type ```echo $ROS_IP```

    - If you are not running ROS services in a Docker container, replace the ROS IP Address value with the IP address of your ROS machine. Ensure that the ```Host Port``` is set to ```10000```.

    - If you are running ROS services in a ```Docker``` container, fill ROS IP Address with the loopback IP address ```127.0.0.1```.

- The other settings can be left as their defaults. Opening the ROS Settings has created a ROSConnectionPrefab in  ```Assets/Resources ``` with the user-input settings. When the static  ```ROSConnection.instance ``` is referenced in a script, if a  ```ROSConnection ``` instance is not already present, the prefab will be instantiated in the Unity scene, and the connection will begin.

> Note: While using the ROS Settings menu is the suggested workflow as of this version, you may still manually create a GameObject with an attached ROSConnection component.

# User Input
- Next, we will add a UI element that will allow user input to trigger the  ```Publish() ``` function. In the Hierarchy window, right click to add a new UI > Button. Note that this will also create a new Canvas parent, as well as an Event System.
> Note: In the  ```Game ``` view, you will see the button appear in the bottom left corner as an overlay. In  ```Scene ``` view the button will be rendered on a canvas object that may not be visible.

> Note: In case the Button does not start in the bottom left, it can be moved by setting the  ```Pos X ``` and  ```Pos Y ``` values in its Rect Transform component. For example, setting its Position to  ```(-200, -200, 0) ``` would set its position to the bottom right area of the screen.

- Select the newly made Button object, and scroll to see the Button component in the Inspector. Click the  ```+ ``` button under the empty ```OnClick()``` header to add a new event. Select the  ```Publisher``` object in the Hierarchy window and drag it into the new  ```OnClick()``` event, where it says  ```None (Object)```. Click the dropdown where it says  ```No Function ```. Select  ```GenericPublisher > ```  ```Publish() ```.

- To change the text of the Button, expand the Button Hierarchy and select Text. Change the value in Text on the associated component.

# On the ROS Side
- In Ubuntu we also need the required framework, run the script ```3_ros_unity_integration.sh```. This will clone and build the ROS-TCP-Endpoint Repository.

-  To enable this connection we need to set our ROS_IP and ROS_TCP_PORT parameters, which are read from the file ```src/ros_tcp_endpoint/config/params.yaml```. Your ROS_IP may be added to the config via the following command:

```shell
echo "ROS_IP: $ROS_IP" > ~/catkin_ws/src/ROS-TCP-Endpoint/config/params.yaml
```
> Note: You may need to modify this file if the IP address of your ROS machine changes

Because I don't want to have to do this every time I restart WSL, I added the following two lines to the ```/usr/home/.bashrc``` 

```shell
string="ROS_IP: $ROS_IP"
sed -i "1s/.*/$string/" ~/catkin_ws/src/ROS-TCP-Endpoint/config/params.yaml
```

- Now you're able to start the ```ros_tcp_endpoint``` via its launch file:

```shell
roslaunch ros_tcp_endpoint endpoint.launch
```

# Test

- Now its time to see everything in action. In Ubuntu start the end point with the command above. Then start the Unity Scene by clicking the play button on top. 

- Now open a new ubuntu terminal and start a listener on the topic you specified, in this example:

```shell
rostopic echo /unity/wrist_position
```

- If you click on the button in Unity, you should see the position and orientation of the robot link being published to the terminal.

- If you want to continously pass the position from Unity to Ros you can also remove the button and rename the ```Publish()``` function in the ```GenericPublisher.cs``` script to ```Update()```. Functions with this name are called in Unity on every update, hence the ROS message will be continously be sent over to your ROS machine. 

> Note: You can check the framerate with ```rostopic hz /unity/writst_position``` 

