import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm
 
# Creating a series of data of in range of 1-50.
x = np.linspace(6,7,200)
 
'''
Merge 2 Gaussians
'''
def merge_gaussian(m1, m2, sd1, sd2):
    #New mean
    m12 = m1*(sd2**2/(sd1**2+sd2**2)) + m2*(sd1**2/(sd1**2+sd2**2))

    #New standard deviation
    sd12 = np.sqrt(sd1**2*sd2**2/(sd1**2+sd2**2))

    return m12, sd12

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

#Random measurements
randnums1 = np.random.uniform(6,7,50)
randnums2 = np.random.uniform(6,6.9,100)

#Mean and Standard devaition of the camera
mean1, sd1 = mean_variance(randnums1)
mean2, sd2 = mean_variance(randnums2)
 
#Apply function to the data.
mean12, sd12 = merge_gaussian(mean1, mean2, sd1, sd2)

print(f"mean {round(mean12,2)} variance {round(sd12,2)}")
 
#Plotting the Results
plt.plot(x, norm.pdf(x,mean1,sd1), color = 'red')
plt.plot(x, norm.pdf(x,mean2,sd2), color = 'blue')
plt.plot(x, norm.pdf(x,mean12,sd12), color = 'purple')


#Plot vertical lines
plt.vlines(ymin = 0, ymax = max(norm.pdf(x,mean1,sd1)), x = mean1, color = 'red', linestyle = ':', linewidth=1)
plt.vlines(ymin = 0, ymax = max(norm.pdf(x,mean2,sd2)), x = mean2, color = 'blue', linestyle = ':', linewidth=1)
plt.vlines(ymin = 0, ymax = max(norm.pdf(x,mean12,sd12)), x = mean12, color = 'purple', linestyle = ':', linewidth=1)
plt.xlabel('Diameter in cm')
plt.ylabel('Probability Density')

plt.show()