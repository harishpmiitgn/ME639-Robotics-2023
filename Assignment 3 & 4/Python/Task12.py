# Inverse kinematics of Stanford type Manipulator robot

from math import *

print("Enter the co-ordinates of end effector w.r.t. base frame")
x = float(input("x = "))
y = float(input("y = "))
z = float(input("z = "))

l1 = float(input("length of link 1 (l1) = "))
l2 = float(input("length of link 2 (l2) = "))
d =  float(input("height of link 3 (d)  = "))


q2 = acos ((z-l1)/d)
q1 = asin(((-y*d*sin(q2))-(x*l2))/((l2**2)+((d**2)*(sin(q2)**2))))

q2 = round(degrees(q2),2)
q1 = round(degrees(q1),2)
print("q1 = ",q1)
print("q2 = ",q2)

