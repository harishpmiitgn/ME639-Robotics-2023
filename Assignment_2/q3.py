import numpy as np

def getH(R, d):
    H=[[R[0][0],R[0][1], R[0][2],d[0][0]],
       [R[1][0],R[1][1], R[1][2],d[1][0]],
       [R[2][0],R[2][1], R[2][2],d[2][0]],
       [0,0, 0,1]]
    return H

def getR(q):
    R=[[np.cos(q), -np.sin(q), 0],
       [np.sin(q), np.cos(q), 0],
       [0, 0, 1]]
    return R

l1=float(input("enter length of first link "))
l2=float(input("enter length of second link "))
l3=float(input("enter length of third link "))
l4=float(input("enter minimum definite length extension of prismatic joint "))
d=float(input("enter length extended by the prismatic joint "))
q1=float(input("enter the angle turned by the first joint "))
q2=float(input("enter the angle turned by the second joint "))

q1=np.radians(q1)
q2=np.radians(q2)

H01=getH(getR(q1), [[0],[0],[0]])
H12=getH(getR(q2), [[0],[l2],[l1]])
H23=getH(getR(0), [[0],[l3],[-l4]])
P=np.linalg.multi_dot([H01,H12,H23,[[0],[0],[-d],[1]]])

print(f"\nfinal position of the end effector is {P[0][0].round(decimals=2)}i {P[1][0].round(decimals=2)}j {P[2][0].round(decimals=2)}k with respect to the base frame\n")
