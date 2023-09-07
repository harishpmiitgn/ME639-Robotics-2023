import numpy as np

l1=float(input("enter length of first link "))
l2=float(input("enter length of second link "))
l3=float(input("enter length of third link "))
q1=float(input("enter the angle turned by the first joint "))
q2=float(input("enter the angle turned by the second joint "))
q3=float(input("enter the angle turned by the third joint "))

J=[[-l3*np.sin(q1+q2+q3)-l2*np.sin(q1+q2)-l1*np.sin(q1),-l3*np.sin(q1+q2+q3)-l2*np.sin(q1+q2), -l3*np.sin(q1+q2+q3) ],
   [l3*np.cos(q1+q2+q3)+l2*np.cos(q1+q2)+l1*np.cos(q1),l3*np.cos(q1+q2+q3)+l2*np.cos(q1+q2), l3*np.cos(q1+q2+q3)],
   [0,0,0],
   [0,0,0],
   [0,0,0],
   [1,1,1]]

print(f"the manipulator jacobian for the given configuration is \n|{J[0][0].round(2)}  {J[0][1].round(2)}  {J[0][2].round(2)}|\n|{J[1][0].round(2)}  {J[1][1].round(2)}  {J[1][2].round(2)}|\n|{J[2][0]}     {J[2][1]}     {J[2][2]}|")