#Harikrishnan R
import numpy as np

def getH(R, d):                             #homogeneous transformation
    H=[[R[0][0],R[0][1], R[0][2],d[0][0]],
       [R[1][0],R[1][1], R[1][2],d[1][0]],
       [R[2][0],R[2][1], R[2][2],d[2][0]],
       [0,0, 0,1]]
    return H

def getR(q):                                    #rotation mat about z
    R=[[np.cos(q), -np.sin(q), 0],
       [np.sin(q), np.cos(q), 0],
       [0, 0, 1]]
    return R

def getRx(q):                                    #rotation mat about x
    Rx=[[1, 0, 0],
        [0, np.cos(q), -np.sin(q)],
        [0, np.sin(q), np.cos(q)]]
    return Rx


### inputs
l1=float(input("enter length of 1st link "))
l2=float(input("enter length of 2nd link "))
l3=float(input("enter length of 3rd link "))
l4=float(input("enter length of 4th link "))
l5=float(input("enter length of 5th link "))
l6=float(input("enter length of 6th link "))
l7=float(input("enter length of fixed end effector displacement "))

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
H01=getH(getR(q1), [[0],[0],[l1]])
H12=getH(np.matmul(getRx(np.pi/2),getR(q2)), [[0],[0],[0]])
H23=getH(getR(q3), [[l2],[0],[0]])
H34=getH(getR(q4), [[l3],[0],[-l4]])
H45=getH(np.matmul(getRx(np.pi/2),getR(q5)), [[0],[-l5],[0]])
H56=getH(np.matmul(getRx(-np.pi/2),getR(q6)), [[0],[-l6],[0]])
P=np.linalg.multi_dot([H01,H12,H23,H34,H45,H56,[[0],[0],[-l7],[1]]])

#print outputs
print(f"\nfinal position of the end effector is {P[0][0].round(decimals=2)}i {P[1][0].round(decimals=2)}j {P[2][0].round(decimals=2)}k with respect to the base frame\n")


####### OUTPUTS #########
# for all link lengths =1 and all angles of rotation 0,
# position of end effector is 2i, 3j, 0k which aligns with initial position of end effector 