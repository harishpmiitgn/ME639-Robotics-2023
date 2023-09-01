import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
L1 =int(input("enter length of link 1= "))
L2=int(input("enter length of link 2= "))
a=L1+L2
fig, ax = plt.subplots()
ax.set_xlim(-a, a)
ax.set_ylim(-a/2, a)
plt.grid()

for t1 in range(35, 145,1):
    for t2 in range(35, 145,1):
        x=L1*np.cos(t1*(np.pi/180))+L2*np.cos((t1+t2)*(np.pi/180))
        y=L1*np.sin(t1*(np.pi/180))+L2*np.sin((t1+t2)*(np.pi/180))
        plt.scatter(x,y, s=1, c='b')
plt.text(-a/2,-a/4,"t1,t2 are in range (35,145)")
plt.text(-a/2,-a/3,"t2 is measured wrt t1")
plt.title("workspace of 2R Manipulator")
plt.show()
