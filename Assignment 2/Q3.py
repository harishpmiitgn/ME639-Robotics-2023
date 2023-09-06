from math import *
import numpy as np

# System information

l1 = float(input("l1 = "))     # length of link 1
h1 = float(input("h1 = "))     # height of link 1
l2 = float(input("l2 = "))     # length of link 2
l3 = float(input("l3 = "))     # length of prismatic arm 
d  = float(input("d  = "))     # Extended length of prismatic arm 

# User input 

q1 = int(input("q1 = "))      # Joint 1 angle
q1 = radians(q1)
q2 = int(input("q2 = "))      # Joint 2 angle
q2 = radians(q2)


# Transformation matrix

H01 = np.array([[cos(q1), -sin(q1), 0, l1*cos(q1)],
                [sin(q1),  cos(q1), 0, l1*sin(q1)],
                [0,        0,       1, h1        ],
                [0,        0,       0, 1         ]])


H12 = np.array([[cos(q2), -sin(q2), 0, l2*cos(q2)],
                [sin(q2),  cos(q2), 0, l2*sin(q2)],
                [0,        0,       1, 0         ],
                [0,        0,       0, 1         ]])

H23 = np.array([[1, 0, 0,  0   ],
                [0, 1, 0,  0   ],
                [0, 0, 1, -l3  ],
                [0, 0, 0,  1   ]])

P3 = np.array([[ 0 ],
               [ 0 ],
               [-d ],
               [ 1 ]])

P0 = np.matmul(np.matmul(np.matmul(H01,H12),H23),P3)

P_0x = P0[0][0]
if(abs(P_0x)<(1e-5)):
    P_0x = 0.0
P_0y = P0[1][0]
if(abs(P_0y)<(1e-5)):
    P_0y = 0.0
P_0z = P0[2][0]
if(abs(P_0z)<(1e-5)):
    P_0z = 0.0

print(" P_0x = ",P_0x)
print(" P_0y = ",P_0y)
print(" P_0z = ",P_0z)