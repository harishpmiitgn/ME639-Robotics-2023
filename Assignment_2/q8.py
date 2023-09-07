import numpy as np

l1=float(input("enter length of first link "))
l2=float(input("enter length of second link "))
l3=float(input("enter length of third link "))
l4=float(input("enter minimum definite length extension of prismatic joint "))
d=float(input("enter length extended by the prismatic joint "))
q1=float(input("enter the angle turned by the first joint "))
q2=float(input("enter the angle turned by the second joint "))

J=[[-l3*np.cos(q1+q2)-l2*np.cos(q1),-l3*np.cos(q1+q2), 0 ],
   [-l3*np.sin(q1+q2)-l2*np.sin(q1),-l3*np.sin(q1+q2), 0],
   [0,0,1],
   [0,0,0],
   [0,0,0],
   [1,1,0]]

print(f"the manipulator jacobian for the given configuration is \n|{J[0][0].round(2)}  {J[0][1].round(2)}  {J[0][2]}|\n|{J[1][0].round(2)}  {J[1][1].round(2)}  {J[1][2]}|\n|{J[2][0]}     {J[2][1]}     {J[2][2]}|")