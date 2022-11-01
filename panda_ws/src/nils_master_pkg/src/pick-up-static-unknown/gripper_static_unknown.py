#!/usr/bin/env python
import serial
import time
import rospy
from std_msgs.msg import String

port_name = "/dev/ttyACM0"
BAUD = 500000
PORT_TIMEOUT = 0.5
PORT_WRITE_TIMEOUT=1

serial_port = serial.Serial(port_name,BAUD,timeout=PORT_TIMEOUT,write_timeout=PORT_WRITE_TIMEOUT)
serial_port.reset_input_buffer()

gripperOpen = True
msg2Controller = ""
            
def callback(data):
    pubM = rospy.Publisher('/gripper2controller', String, queue_size=1)
    pubM.publish(msg2Controller)

    action = data.data
    switch(action)

def send(string):
  print(f"Send: '{string}'")
  try:
    serial_port.write(string.encode())
    return True

  except serial.SerialTimeoutException:
    print("Send failed!")

  return False
    
def switch(lang):
    global gripperOpen
    global msg2Controller
    if lang == "Are you ready?":
      msg2Controller = "Ready!"
    else:
      if lang == "Ready to grasp" and gripperOpen:
        send("init\n")
        time.sleep(1)
        send("grip 2\n")
        time.sleep(2)
        msg2Controller = "Object Grasped"
        gripperOpen = False
      if lang == "Open gripper" and not gripperOpen:
        send("ctrl stop\n")
        time.sleep(1)
        send("m open\n")
        time.sleep(1)
        msg2Controller = "Gripper opened"
        gripperOpen = True
      if lang == "Good job gripper... We did it!":
        serial_port.close()
        msg2Controller = "Likewise!"

def listener():
  rospy.init_node('gripper_static_unknown', anonymous=True)

  rospy.Subscriber("/controller2gripper", String, callback)

  rospy.spin()

if __name__ == '__main__':
  listener()