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
The following script will then install everything required to simulate and control the robot arm

```shell
cd unity_ros_setup
./2_interbotix_setup.sh
```





