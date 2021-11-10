# Mixed Reality Group Only
- If you do not have git installed, install git for Windows first (https://git-scm.com/download/win)

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