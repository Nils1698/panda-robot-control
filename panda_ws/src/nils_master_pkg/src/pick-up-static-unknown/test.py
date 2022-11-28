#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
import time

plt.axis([0,100,0,5])
plt.ion()
plt.show()
m1 = []
v1 = []
m2 = []
v2 = []
m25 = []
v25 = []

'''
Calculate mean and variance
'''
def mean_variance(data):
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
Merge 2 Gaussians
'''
def merge_gaussian(m1, m2, sd1, sd2):
    #New mean
    m12 = m1*(sd2**2/(sd1**2+sd2**2)) + m2*(sd1**2/(sd1**2+sd2**2))

    #New standard deviation
    sd12 = np.sqrt(sd1**2*sd2**2/(sd1**2+sd2**2))

    return m12, sd12


#Camera Measurements
tennisBallMeasurementsC = [68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 68.28, 67.10, 67.10, 67.10]
woodBlockC =              [37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 37.67, 38.85, 38.85]
canC =                    [52.98, 52.98, 54.15, 54.15, 55.33, 55.33, 55.33, 55.33, 55.33, 55.33, 55.33, 56.51, 56.51, 56.51, 56.51, 56.51, 56.51, 57.68, 57.68, 57.68]

#Gripper Measurements
tennisBallMeasurementsG = [60.11, 59.98, 59.98, 60.37, 60.55, 60.13, 60.02, 60.14, 60.21, 59.94, 60.32, 60.34, 60.03, 60.17, 60.19, 60.11, 60.16, 60.86, 60.19, 60.17]
woodBlockG =              [30.08, 29.80, 29.32, 29.19, 29.30, 29.19, 29.81, 29.61, 29.57, 29.04, 29.60, 29.65, 29.24, 29.39, 29.67, 28.67, 29.16, 29.94, 29.31, 29.11]
canG =                    [63.52, 63.26, 63.48, 63.32, 63.28, 62.99, 63.08, 63.06, 63.04, 63.12, 63.16, 62.84, 62.78, 62.79, 62.79, 62.79, 62.81, 62.51, 62.80, 62.83]

#plt.ion()

m1,v1 = mean_variance(tennisBallMeasurementsC)
m2,v2 = mean_variance(tennisBallMeasurementsG)
m25, v25 = merge_gaussian(m1, m2, v1, v2)
m3,v3 = mean_variance(canC)
m4,v4 = mean_variance(canG)
m45, v45 = merge_gaussian(m3, m4, v3, v4)
m5,v5 = mean_variance(woodBlockG)
m6,v6 = mean_variance(canG)
m65, v65 = merge_gaussian(m5, m6, v5, v6)

print(f"m1 {m1} m2 {m2} m25 {m25} v1 {v1} v2 {v2} v25 {v25} ")
x = np.linspace(min(m1, m2, m25)-3,max(m1, m2, m25)+3,20000)
plt.axis([min(m1, m2, m25)*0.98,max(m1, m2, m25)*1.02,0,max(norm.pdf(m1,m1,v1),norm.pdf(m2,m2,v2),norm.pdf(m25,m25,v25))*1.05])

print(norm.pdf(m1,m1,v1))

plt.plot(x, norm.pdf(x,m1,v1))
plt.plot(x, norm.pdf(x,m2,v2))
plt.plot(x, norm.pdf(x,m25,v25))
plt.draw()
plt.pause(0.001)
time.sleep(5)
plt.clf()

x = np.linspace(min(m3, m4, m45)-3,max(m3, m4, m45)+3,20000)
plt.axis([min(m3, m4, m45)-3,max(m3, m4, m45)+3,0,5])

plt.plot(x, norm.pdf(x,m3,v3))
plt.plot(x, norm.pdf(x,m4,v4))
plt.plot(x, norm.pdf(x,m45,v45))

plt.draw()
plt.pause(0.001)
time.sleep(5)
plt.clf()

x = np.linspace(min(m5, m6, m65)-3,max(m5, m6, m65)+3,20000)
plt.axis([min(m5, m6, m65)-3,max(m5, m6, m65)+3,0,5])

plt.plot(x, norm.pdf(x,m5,v5))
plt.plot(x, norm.pdf(x,m6,v6))
plt.plot(x, norm.pdf(x,m65,v65))

plt.draw()
plt.pause(0.001)
time.sleep(5)
plt.clf()
