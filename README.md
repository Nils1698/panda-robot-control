# panda-robot-control
This repository is made in order to simplify the setup of the Emika Frank Robot Arm at the DTU. It will give instruction concerning the start up proedure, the launching of the controller and set up of the camera driver.
# Table of Contents
1. [Robot User Handbook](#handbook)
2. [Start up procedure](#startUp)
3. [Set up of the repository](#repository)
4. [Create a catkin workspace](#catkinWorkspace)
5. [Launching the MRAC controller](#mracLaunch)
6. [Setup of the camera driver](#cameraDriver)
7. [Debugging](#debugging)


## Robot User Handbook
## Start up procedure
## Set up of the repository
### Required ROS packages for the robot
There are 2 ROS packages needed, namely `libfranka` and `franka-ros` in order to run the controllers designed by other students. Have a look at [this](https://frankaemika.github.io/docs/installation_linux.html) website for installations instructions or write the following command in the terminal:
```
sudo apt install ros-noetic-libfranka ros-noetic-franka-ros
```
## Build a new catkin workspace
If the old worspace does not work is lost or the user wants to build a new workspace please follow this guide.
There is also a guide provided by wiki.ros.org which can be found [here](https://wiki.ros.org/catkin/Tutorials/create_a_workspace).

This tutorial assumes that you have installed catkin and sourced your environment. If you installed catkin via apt-get for ROS noetic, your command would look like this:
```
$ source /opt/ros/noetic/setup.bash
```
Let's create and build a catkin workspace: (you can give the folder a different name than "catkin_ws")
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
Then make sure to source the file "catkin_ws/devel/setup.bash" inside of the bashrc file. In order to do so, write the following to open the .bashrc file and add the line "" in the end of it.
```
$ gedit ~/.bashrc
```
Make sure to save the file before closing it. The write: 
```
source ~/.bashrc
```
or reopen the terminal.
## Launching the MRAC controller
## Setup of the BlueFox3 camera driver

## Debugging
## ROS packages of the repository
### franka-aic
This package has been made by Christian Kampp Kruse in 2022 and modified by Nils Meile. It mainly includes an MRAC controller and AIC controller which can be launched using the launch files. For more detailed information concerning the project Christians's Master thesis can be found here XXXTODOXXX.