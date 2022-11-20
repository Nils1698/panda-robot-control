import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm


#Mean and Standard devaition of the camera
#mean1, sd1 = mean_variance(woodBlockG)
#mean2, sd2 = mean_variance(woodBlockC)
#mean1, sd1 = mean_variance(woodBlockG)
#mean1, sd1 = mean_variance(canG)
#mean2, sd2 = mean_variance(randnums2)

#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import String, Float64

class Gaussian(object):
  def __init__(self):
    #Params
    self.loop_rate = rospy.Rate(10)

    # Creating a series of data of in range of 1-50.
    self.x = np.linspace(28,38,20000)

    #Random measurements
    self.randnums1 = np.random.uniform(6,7,50)
    self.randnums2 = np.random.uniform(6,6.9,100)

    #Gripper Measurements
    self.tennisBallMeasurementsG = [60.11, 59.98, 59.98, 60.37, 60.55, 60.13, 60.02, 60.14, 60.21, 59.94, 60.32, 60.34, 60.03, 60.17, 60.19, 60.11, 60.16, 60.86, 60.19, 60.17]
    self.woodBlockG =              [30.08, 29.80, 29.32, 29.19, 29.30, 29.19, 29.81, 29.61, 29.57, 29.04, 29.60, 29.65, 29.24, 29.39, 29.67, 28.67, 29.16, 29.94, 29.31, 29.11]
    self.canG =                    [63.52, 63.26, 63.48, 63.32, 63.28, 62.99, 63.08, 63.06, 63.04, 63.12, 63.16, 62.84, 62.78, 62.79, 62.79, 62.79, 62.81, 62.51, 62.80, 62.83]

    #Camera Measurements
    self.tennisBallMeasurementsC = [68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 67.10, 67.10, 67.10]
    self.woodBlockC =              [37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 38.85, 38.85]
    self.canC =                    [52.98, 52.98, 54.15, 54.15, 55.33, 55.33, 55.33, 55.33, 55.33, 55.33, 55.33, 56.51, 56.51, 56.51, 56.51, 56.51, 56.51, 57.68, 57.68, 57.68]

    #Subscribers
    rospy.Subscriber("/controller2gripper", String, self.callback)
    
    #Publishers
    self.pubM = rospy.Publisher('/objWidthC', Float64, queue_size=1000)

  '''
  Actual decision making process
  '''
  def switch(self, lang):
    #Ready
    if False:
        pass
          

    def plot(self):
        pass
        '''
        #Apply function to the data.
        mean12, sd12 = merge_gaussian(mean1, mean2, sd1, sd2)

        print(f"mean {round(mean12,2)} variance {round(sd12,2)}")
        
        #Plotting the Results
        plt.plot(x, norm.pdf(x,mean1,sd1), color = 'red')
        plt.plot(x, norm.pdf(x,mean2,sd2), color = 'blue')
        #plt.plot(x, norm.pdf(x,mean2,sd2), color = 'blue')
        plt.plot(x, norm.pdf(x,mean12,sd12), color = 'purple')


        #Plot vertical lines
        plt.vlines(ymin = 0, ymax = max(norm.pdf(x,mean1,sd1)), x = mean1, color = 'red', linestyle = ':', linewidth=1)
        plt.vlines(ymin = 0, ymax = max(norm.pdf(x,mean2,sd2)), x = mean2, color = 'blue', linestyle = ':', linewidth=1)
        plt.vlines(ymin = 0, ymax = max(norm.pdf(x,mean12,sd12)), x = mean12, color = 'purple', linestyle = ':', linewidth=1)
        plt.xlabel('Diameter in cm')
        plt.ylabel('Probability Density')

        plt.show()'''

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
        print(f"mean {round(mean,2)} variance {round(variance,2)}")
        return mean, variance
        

    '''
    Subscribes to the controller topic
    '''
    def callback(self, data):
        self.action = data.data


    def start(self):
        while not rospy.is_shutdown():

            self.switch(self.action)
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node("gaussian", anonymous=True)
    my_node = Gaussian()
    my_node.start()