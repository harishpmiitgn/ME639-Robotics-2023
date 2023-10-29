# Inverse kinematics of SCARA Manipulator robot

from math import *

print("Enter the co-ordinates of end effector w.r.t. base frame")
x = float(input("x = "))
y = float(input("y = "))
z = float(input("z = "))

d1 = float(input("height of link 1 (d1)  = "))
l1 = float(input("length of link 1 (l1) = "))
l2 = float(input("length of link 2 (l2) = "))
d = d1 - z

q2 = acos ((x**2+y**2-l1**2-l2**2)/(2*l1*l2))
q1 = atan(y/x) - atan((l2*sin(q2))/(l1+l2*cos(q2)))

q2 = round(degrees(q2),2)
q1 = round(degrees(q1),2)
print("q1 = ",q1)
print("q2 = ",q2)


