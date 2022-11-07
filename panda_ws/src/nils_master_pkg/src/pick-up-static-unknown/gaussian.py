import numpy as np
import matplotlib.pyplot as plt
 
# Creating a series of data of in range of 1-50.
x = np.linspace(5,8,200)
 
#Creating a Function.
def normal_dist(x , mean , sd):
    prob_density = (np.pi*sd) * np.exp(-0.5*((x-mean)/sd)**2)
    return prob_density
 
#Mean and Standard devaition of the camera
width1 = 6.7
sd1 = 0.41 #Todo

#Mean and Standard devaition of the gripper
width2 = 6.4
sd2 = 0.4 #Todo
 
#Apply function to the data.
pdf1 = normal_dist(x, width1, sd1)
pdf2 = normal_dist(x, width2, sd2)

#Calculating the intersection
cross1 = normal_dist(width2, width1 , sd1)
cross2 = normal_dist(width1, width2 , sd2)

if cross1 > cross2:
    print(f"The more correct value with a probability of {round(cross1,2)}% is {width1}cm.")
else:
    print(f"The more correct value with a probability of {round(cross2,2)}% is {width2}cm.")
 
#Plotting the Results
plt.plot(x,pdf1 , color = 'red')
plt.plot(x,pdf2 , color = 'blue')
#Plot horizontal lines
plt.hlines(y = cross1, xmin = 5, xmax = width2, color = 'red', linestyle = ':')
plt.hlines(y = cross2, xmin = 5, xmax = width1, color = 'blue', linestyle = ':')
#Plot vertical lines
plt.vlines(ymin = 0, ymax = max(pdf1), x = width1, color = 'grey', linestyle = ':', linewidth=0.5)
plt.vlines(ymin = 0, ymax = max(pdf2), x = width2, color = 'grey', linestyle = ':', linewidth=0.5)
plt.vlines(ymin = 0, ymax = cross1, x = width2, color = 'red', linestyle = ':')
plt.vlines(ymin = 0, ymax = cross2, x = width1, color = 'blue', linestyle = ':')
plt.xlabel('Data points')
plt.ylabel('Probability Density')

plt.show()