import numpy as np
import math

a1 = 1
a2 = 1
d4 = 1

#End-effector position input
p = [0, 0, 0]
n = 0
while n < 3:
    p[n] = int(input())
    n = n+1

#Taking Rotation Matrix as input
#Taking Rotation Matrix as input
R = []

for i in range(3):          # A for loop for row entries
    a =[]
    for j in range(3):      # A for loop for column entries
        a.append(int(input()))
    R.append(a)

#Array to store joint angles 
q = [0, 0, 0]

r = (p[0]**2 + p[1]**2 - a1**2 - a2**2)/2*a1*a2
q[1] = math.atan(math.sqrt(1-r**2), r)
q[0] = math.atan(p[0], p[1]) - math.atan(a1 + a1*np.cos(q[1]), a2*np.sin(q[1]))
alpha = math.atan(R[0][1], R[0][0])
q[2] = q[1] + q[0] - alpha
d3 = p[2] + d4
