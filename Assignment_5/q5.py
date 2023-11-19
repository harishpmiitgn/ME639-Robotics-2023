# spherical wrist, oriented such that first rotational axis along base frame z, second rotational axis about base frome x
# we only consider the wrist, can be coupled with a manipulator such that wrist base frame is manipulator's previous end effector

import numpy as np

def getR(q):                                    #rotation mat about z
    R=[[np.cos(q), -np.sin(q), 0],
       [np.sin(q), np.cos(q), 0],
       [0, 0, 1]]
    return R

print("please enter end effector coordinates ")
x=float(input())
y=float(input())
z=float(input())

print("please enter the link lengths")
l1=float(input())
l2=float(input())
l3=float(input())

q1 = np.arctan2(x,y)

x_ = l1*np.cos(q1)

theta=np.arccos( ((x-x_)**2 + (z-l1)**2-l2**2 - l3**2)/(2*(l3*l2)) )
q2=np.arctan2((z-l1),(x-x_))-np.arctan2( (l3*np.sin(theta)),(l2+l3*np.cos(theta)) )
q3= theta

print(f" inverse solutions are q1 = {np.degrees(q1)}, q2 = {np.degrees(q2)}, q3 = {np.degrees(q3)}")

R01 = getR(q1)
R12 = np.matmul([[1, 0, 0],[0, 0, -1],[0, 1, 0]], getR(q2))
R23 = getR(0)

R03= np.linalg.multi_dot([R01, R12, R23])

print("please enter orientation matrix R: ")

R_=[[0, 0, 0],
    [0, 0, 0],
    [0, 0, 0]]

R_[0][0]= input()
R_[0][1]= input()
R_[0][2]= input()

R_[1][0]= input()
R_[1][1]= input()
R_[1][2]= input()

R_[2][0]= input()
R_[2][1]= input()
R_[2][2]= input()

R36 = np.matmul((R03.T), R_)
r=R36

q4 = np.arctan2(np.cos(q1)*np.cos(q2+q3)*r[0][2] + np.sin(q1)*np.cos(q2+q3)*r[1][2] + np.sin(q2+q3)*r[2][2], -np.cos(q1)*np.sin(q2+q3)*r[0][2] - np.sin(q1)*np.sin(q2+q3)*r[1][2] + np.cos(q2+q3)*r[2][2]) 
q5 = np.arctan(np.sin(q1)*r[0][2]-np.cos(q1)*r[1][2]+np.sqrt(1-(np.sin(q1)*r[0][2]-np.cos(q1)*r[1][2])**2))
q6 = np.arctan2(-np.sin(q1)*r[0][0]+np.cos(q1)*r[1][0], np.sin(q1)*r[0][1]-np.cos(q1)*r[1][1])


print(f" inverse solutions are q4 = {np.degrees(q4)}, q5 = {np.degrees(q5)}, q6 = {np.degrees(q6)}")