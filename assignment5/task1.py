#for stanford
import numpy as np

d1=1.0
d2=1.0

def InvKin(pos):
    x,y,z=pos
    theta1=np.arctan(y/x)+np.pi*(x<0)
    r=np.sqrt(x**2+y**2)
    s=d1-z
    theta2=np.arctan(r/s)
    if x<0:
        theta2=np.pi-theta2
    d3=np.sqrt(r**2 + s**2)-d2
    return np.array([theta1,theta2,d3])

pos=[1,1,1]

print(InvKin(pos))
