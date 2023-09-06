#!/usr/bin/env python
# coding: utf-8

# TASK 3,4

# In[1]:


import numpy as np

def getH(R, d):                             #homogeneous transformation
    H=[[R[0][0],R[0][1], R[0][2],d[0][0]],
       [R[1][0],R[1][1], R[1][2],d[1][0]],
       [R[2][0],R[2][1], R[2][2],d[2][0]],
       [0,0, 0,1]]
    return H

#rotation matrix about z
def getR(q):                                   
    R=[[np.cos(q), -np.sin(q), 0],
       [np.sin(q), np.cos(q), 0],
       [0, 0, 1]]
    return R

#rotation matrix about x
def getRx(q):                                    
    Rx=[[1, 0, 0],
        [0, np.cos(q), -np.sin(q)],
        [0, np.sin(q), np.cos(q)]]
    return Rx


### inputs
#asking the offsets as inputs for generalization.
d1 = float(input("Enter offset for link 1:"))
d4 = float(input("Enter offset for link 4:"))
d5 = float(input("Enter offset for link 5:"))
d6 = float(input("Enter offset for link 6:"))

r2 = float(input("Enter the link 2 length:"))  # refer the diagram in the answer sheet for reference.
r3 = float(input("Enter the link 3 length:"))

q1=float(input("enter the angle turned by the 1st joint "))
q2=float(input("enter the angle turned by the 2nd joint "))
q3=float(input("enter the angle turned by the 3rd joint "))
q4=float(input("enter the angle turned by the 4th joint "))
q5=float(input("enter the angle turned by the 5th joint "))
q6=float(input("enter the angle turned by the 6th joint "))

##  unit conversion
q1=np.radians(q1)
q2=np.radians(q2)
q3=np.radians(q3)
q4=np.radians(q4)
q5=np.radians(q5)
q6=np.radians(q6)

## definitions
# first parameter is rotation matrix, second is d vector
H01=getH(getR(q1), [[0],[0],[d1]])
H12=getH(np.matmul(getRx(np.pi/2),getR(q2)), [[0],[0],[0]])
H23=getH(getR(q3), [[r2],[0],[0]])
H34=getH(getR(q4), [[r3],[0],[-d4]])
H45=getH(np.matmul(getRx(np.pi/2),getR(q5)), [[0],[d5],[0]])
H56=getH(np.matmul(getRx(-np.pi/2),getR(q6)), [[0],[-d6],[0]])
P=np.linalg.multi_dot([H01,H12,H23,H34,H45,H56,[[0],[0],[2],[1]]]) # assuming the end effector moves 2 units.

#print outputs
print(f"\nfinal position of the end effector is {P[0][0].round(decimals=2)}i {P[1][0].round(decimals=2)}j {P[2][0].round(decimals=2)}k with respect to the base frame\n")


#TASK4 
"""
Entering all the values of link lengths as 2, offsets as 0.2 and angles = 0 as an example , we get the position of the
end tip or the end effector as 4.0i -1.6j 0.4k with respect to the base frame
"""


# TASK 5,6

# In[ ]:


import numpy as np
import math 

#asking the offsets as inputs for generalization.
d1 = float(input("Enter offset for link 1:"))
d4 = float(input("Enter offset for link 4:"))
d5 = float(input("Enter offset for link 5:"))
d6 = float(input("Enter offset for link 6:"))

r2 = float(input("Enter the link 2 length:"))  # refer the diagram in the answer sheet for reference.
r3 = float(input("Enter the link 3 length:"))

#joint angles 
q1 = math.degrees(float(input("angle of link 1: ")))
q2 = math.degrees(float(input("angle of link 2: ")))
q3 = math.degrees(float(input("angle of link 3: ")))
q4 = math.degrees(float(input("angle of link 4: ")))
q5 = math.degrees(float(input("angle of link 5: ")))
q6 = math.degrees(float(input("angle of link 6: ")))

J = []

