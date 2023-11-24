from math import *
print("co-ordinates of end effector with base frame")
x = float(input("x = "))
y = float(input("y = "))
z = float(input("z = "))

L1 = float(input("link 1 length = "))
L2 = float(input("link 2 length = "))
d =  float(input("link 3 length = "))

q1 = asin(((-y*d*sin(q2))-(x*l2))/((l2**2)+((d**2)*(sin(q2)**2))))
q2 = acos ((z-l1)/d)

q1 = round(degrees(q1),2)
q2 = round(degrees(q2),2)
print("q1 = ",q1)
print("q2 = ",q2)

