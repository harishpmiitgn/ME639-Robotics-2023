from math import *
import numpy as np

# Define DH Parameters

d1 = 0
d2 = 0

theta1 = int(input("theta1 = "))      # Joint 1 angle
theta1 = radians(theta1)
theta2 = int(input("theta2 = "))      # Joint 2 angle
theta2 = radians(theta2)

l1 = 91.88     # length of link 1 (mm)
l2 = 104.54    # length of link 2 (mm)

alpha1 = 0
alpha2 = 0

# Function to calculate the homogeneous transformation matrix using DH parameters
def homogenous_Transformation(d,theta,a,alpha):
    H = np.array([[cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
                  [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
                  [0         ,  sin(alpha)           ,  cos(alpha)           , d           ],
                  [0         ,  0                    ,  0                    , 1           ]])
    return H

# Homogeneous Transformation matrix
H01 = homogenous_Transformation(d1,theta1,l1,alpha1)
H12 = homogenous_Transformation(d2,theta2,l2,alpha2)
H02 = np.matmul(H01,H12)

# Print Transformation matrix of end effector
print("\n")
print("Transformation matrix = ")
print(H02)
print("\n")


# Position of end effector

P2 = np.array([[0],
               [0],
               [0],
               [1]])

P0 = (np.matmul(H02,P2))
P0 = P0[:3]

print("End Effector position = ")
print(P0)
print("\n")

# Calculating Jacobian 
O0 = np.array([[0],
               [0],
               [0]])


R00 = np.array([[1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]])

k = np.array([[0],
              [0],
              [1]])

def z(R):
    z_i = np.matmul(R,k) 
    return z_i


O1 = H01[:3, 3:]
O2 = H02[:3, 3:]

R01 = H01[:3, :3]
R02 = H02[:3, :3]

Z0 = z(R00)
Z1 = z(R01)
Z2 = z(R02)

# Calculating Columns J1, J2 of jacobian matrix
J1 = np.zeros((6, 1))
J1_v = np.cross(Z0.flatten(), (O2 - O0).flatten())
J1_w = Z0
J1[:3, 0] = J1_v
J1[3:, 0] = J1_w.flatten()

J2 = np.zeros((6, 1))
J2_v = np.cross(Z1.flatten(), (O2 - O1).flatten())
J2_w = Z1
J2[:3, 0] = J2_v
J2[3:, 0] = J2_w.flatten()

J = np.hstack((J1, J2))
print("Jacobian matrix = ")
print(J)
