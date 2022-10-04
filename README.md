# panda-robot-control
This repository is made in order to simplify the setup of the Emika Frank Robot Arm at the DTU. It will give instruction concerning the start up proedure, the launching of the controller and set up of the camera driver.
# Table of Contents
1. [Robot User Handbook](#handbook)
2. [Start up procedure](#startUp)
3. [Cloneing of the repository](#repository)
4. [Create a catkin workspace](#catkinWorkspace)
5. [Launching the MRAC controller](#mracLaunch)
6. [Setup of the camera driver](#cameraDriver)
7. [Debugging](#debugging)


## Robot User Handbook
## Start up procedure
## Cloneing of the repository
## Build a new catkin workspace
If the old worspace does not work is lost or the user wants to build a new workspace please follow this guide.
There is also a guide provided by wiki.ros.org which can be found here [here](https://wiki.ros.org/catkin/Tutorials/create_a_workspace).

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
## Launching the MRAC controller
## Setup of the camera driver
## Debugging