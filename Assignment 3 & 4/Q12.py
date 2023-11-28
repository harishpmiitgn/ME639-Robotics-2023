import numpy as py
import math


l1 = float(input("enter length of first link = "))
l2 = float(input("enter length of second link = "))
x = float(input("x end effector position = "))
y = float(input("y end effector position = "))
z = float(input("z end effector position = "))


theta1 = np.arctan(y/x)

thetad1 = np.degrees(theta1)

r = (x**2 + y**2)**0.5
s = z - l1

theta2 = np.arctan(s/r)
thetad2 = np.degrees(theta2)

d3 = abs((r**2 + s**2)**0.5 - l2)

print(f"theta 1 = {thetad1} degrees")
print(f"theta 2 = {thetad2} degrees")
print(f"d3 = {d3}")


