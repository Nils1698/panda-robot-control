# gripper-software

The gripper-software package has been designed by Jacob Fiskaali Hertz and is used to visualize data aquired by the Graspian Gripper and send control commands to it.

# Table of Contents

1. [Robot User Handbook](#handbook)
2. [Setting up the gripper software](#setup)
3. [Debugging](#debugging)

## Setting up the gripper software

In order to use the Software it is necessary to download the Teenzy software. A guide on how to install the software can be found on the main page under "Setup a ros interface to the gripper". Once this is completed, you can open VisualCode and install the PlatformIO extension. PlatformIO is a cross-platform, cross-architecture, multi-framework professional IDE tool for embedded system and software engineers who write embedded applications.

Make sure to restart VisualCode once the installation is completed. A new menu point should pop up in the sidebar and in the footer you hould see a small 'Home' symbol. Click on it to open the PlatformIO Home page. Here you can navigate to 'Open project' and select the 'gripper-software' folder of this git repo. Be sure to select the more nested folder since there exist 2 'gripper-software' folders.

Next you should be able to build and upload the code. In order to do so select first the correct environment. You can select the environment in the bottom bar. Make sure your current environment is 'env:main (gripper-software)'. Then in the bottom bar press the arrow symbol that is pointing to the left. The code will now be uploaded. If there is an error it most probably because the gripper cannot be detected or you are in the wrong environment.

Once this is done, you can start the gripper software:

```
python GraspianGripperSoftware/SamplerSoftware/__main__.py
```

## Debugging

## Notes

If interval empty it is in seconds
ms = microseconds
us = milliseconds

Timestamp sub sys ms the add any other subscription to see timestamps together with sensor data
Monitor fixed number of lines

## Commands

### Collect sensor data

1. Be sure gripper is connected and has power
2. Go to the main environment
3. Write sub S1 raw
4. set pub-period 100
   get pub-period
5. pub-on
6. Monitor Interval 1s and 20 datapoints -> Start
7. Select x and y samples and set graph

### Moving gripper

1. Be sure gripper is connected and has power
2. Go to the main environment
3. Start graspian software
4. Write m home
5. Write m goto 60 (For 60mm width)

### If program is stuck

Try: ctrl stop -> m home
