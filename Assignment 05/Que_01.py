import numpy as np
import math 

#Input for End-Effector position
p = [0, 0, 0]
n = 0
while n < 3:
    p[n] = int(input())
    n = n+1
# p0 = int(input())
# p1 = int(input())
# p2 = int(input())

def d2r( angle ):
    angle = (angle/180)*3.14
    return angle

def r2d(angle):
    angle = (angle/3.14)*180
    return angle

#Link Parameters
l1 = 1
l2 = 1
l3 = 1

r = np.sqrt(p[0]*p[0] + p[1]*p[1])


theta_1 = np.arctan(p[0]/p[1])
theta_2 = np.arctan(r/p[2])

print(r2d(theta_1))
print(r2d(theta_2))
