# gripper-software

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
4. Monitor Interval 1s and 20 datapoints -> Start
5. Select x and y samples and set graph

### Moving gripper

1. Be sure gripper is connected and has power
2. Go to the main environment
3. Start graspian software
4. Write m home
5. Write m goto 60 (For 60mm width)

### If program is stuck

Try: ctrl stop -> m home
