import numpy as np
import math

def h(theta,d):
    t=np.radians(theta)
    h=np.zeros((4,4))
    h[0][0]=np.cos(t)
    h[0][1]=-np.sin(t)
    h[1][0]=np.sin(t)
    h[1][1]=np.cos(t)
    h[2][2]=1
    h[3][3]=1
    for i in range(3):
        h[i][3]=d[i]
    print(h)
    return h
#link lengths
l1=2
l2=3

q1=float(input("Q1 angle="))
q2=float(input("Q2 angle="))
ed=float(input("Extension l3="))

d01=[0,0,0]
d12=[0,l1,0]
d23=[0,l2,0]
p=np.array([0,0,ed,1])
p=np.transpose(p)

h01=h(q1,d01)
h12=h(q2,d12)
h23=h(0,d23)

op=np.dot(np.dot(h01,h12),np.dot(h23,p))

print("end effector")
print(op[:3])

