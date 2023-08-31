import numpy as np
import math

def calc_r(theta,axi):
    t=np.radians(theta)
    if axi=='z':        
        h=np.zeros((3,3))
        h[0][0]=np.cos(t)
        h[0][1]=-np.sin(t)
        h[1][0]=np.sin(t)
        h[1][1]=np.cos(t)
        h[2][2]=1
    elif axi=='y':
        h=np.zeros((3,3))
        h[0][0]=np.cos(t)
        h[0][2]=np.sin(t)
        h[2][0]=-np.sin(t)
        h[2][2]=np.cos(t)
        h[1][1]=1
    elif axi=='x':
        h=np.zeros((3,3))
        h[1][1]=np.cos(t)
        h[1][2]=-np.sin(t)
        h[2][1]=np.sin(t)
        h[2][2]=np.cos(t)
        h[0][0]=1
    return h
#link lengths
def calc_h(r,d):
    h=np.zeros((4,4))
    for i in range(3):
        for j in range(3):
            h[i][j]=r[i][j]
    h[3][3]=1
    for i in range(3):
        h[i][3]=d[i]
    
    return h
l1=2
l2=3

q1=float(input("Q1 angle="))
q2=float(input("Q2 angle="))
ed=float(input("Extension l3="))

d01=[0,0,0]
d12=[0,0,l1]
d23=[0,l2,0]
p=np.array([0,0,ed,1])
p=np.transpose(p)

h01=calc_h(calc_r(q1,'z'),d01)
h12=calc_h(calc_r(q2,'y'),d12)
h23=calc_h(calc_r(0,'y',d23))

op=np.dot(np.dot(h01,h12),np.dot(h23,p))

print("end effector")
print(op[:3])

