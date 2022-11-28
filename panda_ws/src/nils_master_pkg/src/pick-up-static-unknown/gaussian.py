#!/usr/bin/env python
import rospy
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
from std_msgs.msg import String
from std_msgs.msg import String, Float64

class Gaussian(object):
    def __init__(self):
        #Params
        self.loop_rate = rospy.Rate(10)
        self.initialized = False
        self.msg2Controller = ""
        self.action = ""
        self.cameraWidth = 0.0
        self.gripperWidth = 0.0
        self.arrCameraWidth = []
        self.varG = 0.9070545833333359 #For now

        plt.axis([0,100,0,5])
        plt.ion()
        plt.show()
        
        #Gripper Measurements
        self.tennisBallMeasurementsG = [60.11, 59.98, 59.98, 60.37, 60.55, 60.13, 60.02, 60.14, 60.21, 59.94, 60.32, 60.34, 60.03, 60.17, 60.19, 60.11, 60.16, 60.86, 60.19, 60.17]
        self.woodBlockG =              [30.08, 29.80, 29.32, 29.19, 29.30, 29.19, 29.81, 29.61, 29.57, 29.04, 29.60, 29.65, 29.24, 29.39, 29.67, 28.67, 29.16, 29.94, 29.31, 29.11]
        self.canG =                    [63.52, 63.26, 63.48, 63.32, 63.28, 62.99, 63.08, 63.06, 63.04, 63.12, 63.16, 62.84, 62.78, 62.79, 62.79, 62.79, 62.81, 62.51, 62.80, 62.83]

        #Camera Measurements
        self.tennisBallMeasurementsC = [68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 67.10, 67.10, 67.10]
        self.woodBlockC =              [37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 38.85, 38.85]
        self.canC =                    [52.98, 52.98, 54.15, 54.15, 55.33, 55.33, 55.33, 55.33, 55.33, 55.33, 55.33, 56.51, 56.51, 56.51, 56.51, 56.51, 56.51, 57.68, 57.68, 57.68]

        #Subscribers
        rospy.Subscriber("/controller2gaussian", String, self.callback)
        rospy.Subscriber("/camera2gaussian", Float64, self.callbackCamera)
        rospy.Subscriber("/gripper2gaussian", Float64, self.callbackGripper)
        
        #Publishers
        self.pubM = rospy.Publisher('/gaussian2controller', String, queue_size=10)

    '''
    Actual decision making process
    '''
    def switch(self, lang):
        #Ready
        if lang == "Are you ready?" and not self.initialized:
            self.initialized = True
            self.msg2Controller = "Ready!"

        else:
            if lang == "Reset width":
                self.cameraWidth = 0.0
                self.gripperWidth = 0.0
                self.arrCameraWidth = []
                self.msg2Controller = "Width and Graph reset"
                plt.clf()
            elif lang == "Extract width" and self.dataAquired():
                self.msg2Controller = "Retrieved width estimate"

        self.pubM.publish(self.msg2Controller)
                
          
    '''
    Aquires width measurements
    '''
    def dataAquired(self):
        if self.cameraWidth != 0.0 and len(self.arrCameraWidth) <= 100:
            self.msg2Controller = "Retrieving camera width estimates"
            self.arrCameraWidth.append(self.cameraWidth)
            return False

        elif len(self.arrCameraWidth) >= 100 and self.gripperWidth == 0.0:
            self.msg2Controller = "Retrieved camera width estimate"
            return False

        elif self.gripperWidth != 0.0 and self.msg2Controller == "Retrieved camera width estimate":
            meanG = self.gripperWidth
            meanC, varC = self.mean_variance(self.arrCameraWidth)
            meanN, varN = self.merge_gaussian(meanC, meanG, varC, self.varG)
            if round(varC,2) == 0.0:
                rospy.logwarn(f"Camera variance is 0.0. Is set to 0.01")
                varC = 0.01

            rospy.logwarn(f"Gripper \t{round(meanG,2)}mm {round(self.varG,2)}")
            rospy.logwarn(f"Camera \t{round(meanC,2)}mm {round(varC,2)}")
            rospy.logwarn(f"New \t{round(meanN,2)}mm {round(varN,2)}")

            self.x = np.linspace(min(meanC, meanG, meanN)-3,max(meanC, meanG, meanN)+3,20000)
            plt.axis([min(meanG, meanC, meanN)*0.95,max(meanG, meanC, meanN)*1.05,0,max(norm.pdf(meanG,meanG,self.varG),norm.pdf(meanC,meanC,varC),norm.pdf(meanN,meanN,varN))*1.05])

            plt.plot(self.x, norm.pdf(self.x,meanC,varC), 'b-')
            plt.plot(self.x, norm.pdf(self.x,meanG,self.varG), 'g-')
            plt.plot(self.x, norm.pdf(self.x,meanN,varN), 'r-')
            
            plt.draw()
            plt.pause(0.001)

            return True

    '''
    Merge 2 Gaussians
    '''
    def merge_gaussian(self, m1, m2, sd1, sd2):
        #New mean
        m12 = m1*(sd2**2/(sd1**2+sd2**2)) + m2*(sd1**2/(sd1**2+sd2**2))

        #New standard deviation
        sd12 = np.sqrt(sd1**2*sd2**2/(sd1**2+sd2**2))

        return m12, sd12

    '''
    Calculate mean and variance
    '''
    def mean_variance(self, data):
        # Number of observations
        n = len(data)
        # Mean of the data
        mean = sum(data) / n
        # Square deviations
        deviations = [(x - mean) ** 2 for x in data]
        # Variance
        variance = sum(deviations) / n
        return mean, variance
        

    '''
    Subscribes to the controller topic
    '''
    def callback(self, data):
        self.action = data.data

    '''
    Subscribes to the controller topic
    '''
    def callbackCamera(self, data):
        self.cameraWidth = data.data

    '''
    Subscribes to the controller topic
    '''
    def callbackGripper(self, data):
        self.gripperWidth = data.data

    '''
    Start function
    '''
    def start(self):
        while not rospy.is_shutdown():
            self.switch(self.action)
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("gaussian", anonymous=True)
    my_node = Gaussian()
    my_node.start()