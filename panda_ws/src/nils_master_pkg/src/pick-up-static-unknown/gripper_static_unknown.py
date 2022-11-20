#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import time
import re
import serial

class GripperStaticUnknown(object):
  def __init__(self):
    #Params
    self.loop_rate = rospy.Rate(10)

    self.port_name = "/dev/ttyACM0"
    self.BAUD = 500000
    self.PORT_TIMEOUT = 0.5
    self.PORT_WRITE_TIMEOUT=1

    self.serial_port = serial.Serial(self.port_name,self.BAUD,timeout=self.PORT_TIMEOUT,write_timeout=self.PORT_WRITE_TIMEOUT)
    self.serial_port.reset_input_buffer()

    self.measurements = []
    self.initialized = False
    self.gripperOpen = True
    self.getData = False
    self.objAttachedSys = None
    self.objAttachedCount = []
    self.msg2Controller = ""
    self.last_line = ""
    self.action = ""
    self.tic = 0

    #Subscribers
    rospy.Subscriber("/controller2gripper", String, self.callback)
    
    #Publishers
    self.pubM = rospy.Publisher('/gripper2controller', String, queue_size=1000)

  def send(self, string):
    print(f"Send: {string}")
    try:
      self.serial_port.write(string.encode())
      return True

    except serial.SerialTimeoutException:
      print("Send failed!")

    return False

  '''
  Actual decision making process
  '''
  def switch(self, lang):
    toc = time.perf_counter()
    #print(toc - self.tic)
    #Ready
    if lang == "Are you ready?" and not self.initialized:
      self.send("init\n")
      time.sleep(0.01)
      self.send("sub-clear\n")
      time.sleep(0.01)
      self.send("sub S1 force all\n")
      time.sleep(0.01)
      self.send("sub S2 force all\n")
      time.sleep(0.01)
      self.send("pub-on\n")
      self.initialized = True
      self.msg2Controller = "Ready!"
    else:
      #Ready to grasp
      if lang == "Ready to grasp" and self.gripperOpen:
        time.sleep(0.01)
        self.send("grip 2\n")
        time.sleep(2)
        self.send("ctrl stop\n")
        time.sleep(0.1)
        self.tic = time.perf_counter()
        self.gripperOpen = False
        self.getData = True
        time.sleep(1)

      elif self.objAttachedSys and not self.gripperOpen:
        self.msg2Controller = "Object Grasped"

      if toc - self.tic > 20 and not self.gripperOpen and lang == "Ready to grasp":
        rospy.logwarn("No object found.")
        self.msg2Controller = "Object lost"
        self.getData = False        

      #Lost object
      if lang == "Object grasped" and not self.gripperOpen and self.objAttachedSys == False:
        rospy.logwarn("The object fell down")
        self.msg2Controller = "Object lost"
        self.getData = False

      #Do nothing
      if lang == "Going back to main position":
        self.msg2Controller = "Waiting for instructions"

      #Open Gripper
      if lang == "Open gripper" and not self.gripperOpen:
        self.openGripper()

      #Finished
      if lang == "Good job gripper... We did it!":
        self.serial_port.close()
        time.sleep(0.01)
        self.send("sub-clear\n")
        time.sleep(0.01)
        self.send("pub-off\n")
        self.msg2Controller = "Likewise!"

      #Measuring
      if self.getData:
        self.extractData(toc)
  
  def extractData(self, toc):
      if self.gripperOpen:
        rospy.logwarn("The gripper is not closed!")
      else:
        self.measurements = self.last_line.replace("\n","").split(" ")[:-3]
        self.isObjAttached(self.measurements)
        print(self.measurements)

        if len(self.last_line) == 6 and bool(re.search(r'\d+\.\d+', self.last_line)):
          width = self.last_line.replace("\n","")
          rospy.logwarn(f"The measured object width is {width}mm.")

        try:
          serialString = self.serial_port.readline()
          if(serialString==b''): # time out
            pass
          try:
            self.last_line = serialString.decode("utf-8").replace("\r","").replace("\t"," ")

          except UnicodeDecodeError:
            print("Decoding error: ", end='')
            print(serialString)

        except serial.SerialException:
          print("SensorInterface - Lost conenction!")
        print(round(toc - self.tic,1))
        if round(toc - self.tic,1) == 10.1:
          self.send("m pos\n")

        #self.send("m pos\n")
          
  '''
  Gives information if there is an object attached or not
  '''
  def isObjAttached(self, measurements):
    objAttached = 0
    countAttached = 0

    for i in range(len(measurements)):
      if bool(re.search(r'\d+\.\d+', measurements[i])):
        try:
          if measurements[i] == 'WARNING:':
            objAttached = prevObjAttached
          if float(measurements[i]) > 0.005:
            objAttached = 1
            break
        except ValueError:
          rospy.logwarn(f"A ValueError occured with {measurements[i]}")

    self.objAttachedCount.append(objAttached)

    if len(self.objAttachedCount) > 20:
      self.objAttachedCount.pop(0)

    print(self.objAttachedCount)
    for obj in self.objAttachedCount:
      if obj == 0:
        countAttached += 1
      if obj == 1:
        countAttached -= 1

    if countAttached >= 15:
      self.objAttachedSys = False
    if countAttached <= -15:
      self.objAttachedSys = True

    prevObjAttached = objAttached

  '''
  Function to open the gripper
  '''
  def openGripper(self):
    self.send("m open\n")
    time.sleep(2)
    self.objAttachedCount = [0] * 20
    self.objAttachedCount = []
    self.objAttachedSys = None
    self.getData = False
    self.gripperOpen = True
    self.msg2Controller = "Gripper opened"
    

  '''
  Subscribes to the controller topic
  '''
  def callback(self, data):
    self.action = data.data


  def start(self):
      while not rospy.is_shutdown():
        self.pubM.publish(self.msg2Controller)

        self.switch(self.action)
        self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("gripper_static_unknown", anonymous=True)
    my_node = GripperStaticUnknown()
    my_node.start()