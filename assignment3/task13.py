#for scara
import numpy as np

d1=1.0
d2=1.0
d3=1.0

def InvKin(pos):
    x,y,z=pos
    r_square=(x**2+y**2-d2**2-d3**2)/(2*d2*d3)

    theta2=np.arctan2(np.sqrt((1-r_square)),np.sqrt(r_square))
    theta1=np.arctan2(x,y)-np.arctan2(d2+d3*np.cos(theta2),d3*np.sin(theta2))
    d4=d1-z

    return np.array([theta1,theta2,d4])

pos=[1,1,1]

print(InvKin(pos))
