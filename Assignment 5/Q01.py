import numpy as np
import math 


p = [0, 0, 0]   #end-effector position
n = 0

while n < 3:
    p[n] = int(input())
    n = n+1

def d2r( angle ):
    angle = (angle/180)*3.14
    return angle

def r2d(angle):
    angle = (angle/3.14)*180
    return angle

L1 = 1   # link length
L2 = 1   # link length
L3 = 1   # link length


r = np.sqrt(p[0]*p[0] + p[1]*p[1])

angle_1 = np.arctan(p[0]/p[1])
angle_2 = np.arctan(r/p[2])

print(r2d(angle_1))
print(r2d(angle_2))
