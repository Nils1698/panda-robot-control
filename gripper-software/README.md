# gripper-software

The gripper-software package has been designed by Jacob Fiskaali Hertz and is used to visualize data aquired by the Graspian Gripper and send control commands to it.

# Table of Contents

1. [Robot User Handbook](#handbook)
2. [Setting up the gripper software](#setup)
3. [Debugging](#debugging)

## Setting up the gripper software

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
