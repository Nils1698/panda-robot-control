#!/usr/bin/env python
import serial
import time
import rospy
import re
from std_msgs.msg import String

port_name = "/dev/ttyACM0"
BAUD = 500000
PORT_TIMEOUT = 0.5
PORT_WRITE_TIMEOUT=1

serial_port = serial.Serial(port_name,BAUD,timeout=PORT_TIMEOUT,write_timeout=PORT_WRITE_TIMEOUT)
serial_port.reset_input_buffer()

initialized = False
gripperOpen = True
msg2Controller = ""
last_line = ""
            
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
    global gripperOpen, msg2Controller, last_line, initialized
    if lang == "Are you ready?":
      if not initialized:
        send("init\n")
        time.sleep(0.5)
        send("sub-clear\n")
        initialized = True
      msg2Controller = "Ready!"
    else:
      if lang == "Ready to grasp" and gripperOpen:
        send("sub S1 force all\n")
        time.sleep(0.5)
        send("sub S2 force all\n")
        time.sleep(0.5)
        send("pub-on\n")
        time.sleep(0.5)
        send("grip 2\n")
        time.sleep(2)
        msg2Controller = "Object Grasped"
        gripperOpen = False
      if lang == "Open gripper" and not gripperOpen:
        send("ctrl stop\n")
        time.sleep(0.5)
        send("m open\n")
        time.sleep(0.5)
        msg2Controller = "Gripper opened"
        gripperOpen = True
      if lang == "Good job gripper... We did it!":
        serial_port.close()
        time.sleep(0.5)
        send("sub-clear\n")
        time.sleep(0.5)
        send("pub-off\n")
        msg2Controller = "Likewise!"

      if not gripperOpen:
        print("last_line " + str(last_line))
        measurements = last_line.replace("\n","").split(" ")[:-1]
        #time.sleep(0.001) # yield thread
        try:
          serialString = serial_port.readline()
          if(serialString==b''): # time out
            pass

          try:
            last_line = serialString.decode("utf-8").replace("\r","").replace("\t"," ")

          except UnicodeDecodeError:
            print("Decoding error: ", end='')
            print(serialString)

        except serial.SerialException:
          print("SensorInterface - Lost conenction!")

def listener():
  rospy.init_node('gripper_static_unknown', anonymous=True)

  rospy.Subscriber("/controller2gripper", String, callback)

  rospy.spin()

if __name__ == '__main__':
  listener()