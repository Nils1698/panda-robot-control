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
    pubM = rospy.Publisher('/action2controller', String, queue_size=1)
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
    if lang == "Ready to grasp" and gripperOpen:
        gripperOpen = False
        print("Hello")
        send("init\n")
        time.sleep(1)
        send("grip 2\n")
        time.sleep(2)
        msg2Controller = "Object Grasped"
    if lang == "Open gripper" and not gripperOpen:
        gripperOpen = True
        send("ctrl stop\n")
        time.sleep(1)
        send("m open\n")
        time.sleep(1)
        serial_port.close()
        msg2Controller = "Gripper opened"
    if lang == "Good job gripper... We did it!":
        msg2Controller = "Likewise!"

def listener():
    rospy.init_node('gripper_static_known', anonymous=True)

    rospy.Subscriber("/action2gripper", String, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()