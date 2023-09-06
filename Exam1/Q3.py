# Gajanan Donge
# 20110061

from math import *
import numpy as np

# System information

l1 = float(input("l1 = "))     # length of link 1
l2 = float(input("l2 = "))     # length of link 2
l3 = float(input("l3 = "))     # length of link 3
h3 = float(input("h3 = "))     # height of link 3 
l4 = float(input("l4 = "))     # length of link 4
h4 = float(input("h4 = "))     # height of link 4 
l5 = float(input("l5 = "))     # length of link 5
h5 = float(input("h5 = "))     # height of link 5 
l6 = float(input("l6 = "))     # length of link 6
 

# User input 

q1 = int(input("q1 = "))      # Joint 1 angle
q1 = radians(q1)
q2 = int(input("q2 = "))      # Joint 2 angle
q2 = radians(q2)
q3 = int(input("q3 = "))      # Joint 3 angle
q3 = radians(q1)
q4 = int(input("q4 = "))      # Joint 4 angle
q4 = radians(q2)
q5 = int(input("q5 = "))      # Joint 5 angle
q5 = radians(q1)


# Transformation matrix

H01 = np.array([[-sin(q1), 0, cos(q1), l1*cos(q1)],
                [cos(q1), 0, sin(q1), l1*sin(q1)],
                [0,        1,       0, 0        ],
                [0,        0,       0,  1         ]])


H12 = np.array([[cos(q2), -sin(q2), 0, -l2*sin(q2)],
                [sin(q2),  cos(q2), 0, l2*cos(q2)],
                [0,        0,       1, 0         ],
                [0,        0,       0, 1         ]])

H23 = np.array([[-cos(q3), sin(q3), 0, -h3*sin(q3)],
                [sin(q3),  cos(q3), 0, h3*cos(q3)],
                [0,        0,       -1, l3         ],
                [0,        0,       0, 1         ]])

H34 = np.array([[cos(q4), 0,sin(q4),  h4*sin(q4)],
                [-sin(q4),  0,cos(q4),h4*cos(q4)],
                [0,        -1,       0, l4 ],
                [0,        0,       0, 1   ]])

H45 = np.array([[cos(q5), 0,-sin(q5),  -l5*sin(q5)],
                [sin(q5),  0,cos(q5),l5*cos(q5)],
                [0,        -1,       0, h5 ],
                [0,        0,       0, 1   ]])

P5 = np.array([[ 0 ],
               [ 0 ],
               [l6 ],
               [ 1 ]])

H02 = np.matmul(H01,H12)
H03 = np.matmul(H02,H23)
H04 = np.matmul(H03,H34)
H05 = np.matmul(H04,H45)

P0 = np.matmul(H05,P5)

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
