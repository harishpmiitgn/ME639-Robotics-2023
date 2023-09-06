from math import *
import numpy as np

# System information

l1 = float(input("l1 = "))     # length of link 1
h1 = float(input("h1 = "))     # height of link 1
l2 = float(input("l2 = "))     # length of link 2
l3 = float(input("l3 = "))     # length of prismatic arm 
# d=0

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

H23 = np.array([[1, 0, 0,  0  ],
                [0, 1, 0,  0  ],
                [0, 0, 1, -l3 ],
                [0, 0, 0,  1  ]])


H02 = np.matmul(H01,H12)
H03 = np.matmul(H02,H23)

O0 = np.array([[0],
              [0],
              [0]])

O1 = H01[:3, 3:]
O2 = H02[:3, 3:]
O3 = H03[:3, 3:]

# print (O1)
# print (O2)
# print (O3)

zero = np.array([[0],
                 [0],
                 [0]])

R00 = np.array([[1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]])

R01 = H01[:3, :3]
R02 = H02[:3, :3]
R03 = H03[:3, :3]

K = np.array([[0],
              [0],
              [1]])

Z0 = np.matmul(R00,K)
Z1 = np.matmul(R01,K)
Z2 = np.matmul(R02,K)

J1 = np.zeros((6, 1))
J1_v = np.cross(Z0.flatten(), (O3 - O0).flatten())
J1_w = Z0
J1[:3, 0] = J1_v
J1[3:, 0] = J1_w.flatten()

J2 = np.zeros((6, 1))
J2_v = np.cross(Z1.flatten(), (O3 - O1).flatten())
J2_w = Z1
J2[:3, 0] = J2_v
J2[3:, 0] = J2_w.flatten()

J3 = np.zeros((6, 1))
J3_v = Z2
J3_w = zero
J3[:3, 0] = J3_v.flatten()
J3[3:, 0] = J3_w.flatten()



J = np.hstack((J1, J2, J3))
print(J)
