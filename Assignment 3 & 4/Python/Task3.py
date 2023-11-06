from math import *
import numpy as np

n = int(input("Enter number of links = "))  

# nature of joints (R/P)
while True:
    ask = input("\nDo you want to provide information about the nature of joints? (Y/N): ").upper()
    
    if ask in ['Y', 'N']:
        break
    else:
        print("Error: Invalid input. Please enter 'Y' or 'N'.")

if ask == 'Y':
    joints = []
    print("\nIf joint is a Revolute joint, type 'R'")
    print("If joint is a Prismatic joint, type 'P'\n")
    
    for i in range(n):
        inp = input(f"Type of joint {i + 1}: ")
        if inp == 'R':
            joints.append(1)
        else:
            joints.append(0)
else:
    # Default values ('R') for joints
    joints = [1] * n
#print(joints)

print("\nProvide DH parameters")
DH = np.zeros((n, 4))
for i in range(n):
    print(f"\nFor Link {i + 1}:")
    DH[i, 0] = float(input("d: "))
    DH[i, 1] = radians(float(input("θ (degrees): ")))
    DH[i, 2] = float(input("a: "))
    DH[i, 3] = radians(float(input("α (degrees) : ")))
# print("\nDH Parameters:")
# print(DH)

H = []      # List of homogeneous transformation matrices H_i_i-1 
for i in range(n):
    d = DH[i][0]
    theta = DH[i][1]  
    a = DH[i][2]
    alpha = DH[i][3]

    h = np.array([[cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
                  [sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
                  [0         ,  sin(alpha)           ,  cos(alpha)           , d           ],
                  [0         ,  0                    ,  0                    , 1           ]])
    h = np.round(h, decimals=2)
    H.append(h)
#print(H)

H0 = []        # List of homogeneous transformation matrices H_0_i
H0.append(H[0])
h = H[0]
for i in range(1,n):
    h = np.matmul(h,H[i])
    H0.append(h)
#print(H0)

R = []          # List of Rotation matrices R_0_i
O = []          # List of distance of end point of ith joint from origin

# O0 = zero
zero = np.array([[0],
                 [0],
                 [0]])

O.append(zero)

Z = []
k = np.array([[0],
             [0],
             [1]])

Z.append(k)
for i in range(n):
    r = H0[i][:3,:3]
    o = H0[i][:3, 3:4]
    R.append(r)
    O.append(o)
    z = np.matmul(r,k)
    Z.append(z)
#print(Z)
#print(O)

On = O[n]
J=[]        # List for columns of jacobian matrix for ith link
for i in range(n):
    j = np.zeros((6,1))
    # If revolute joint
    if(joints[i]==1):
        J_v = np.cross(Z[i].flatten(), (On - O[i]).flatten())
        J_w = Z[i]
        j[:3, 0] = J_v
        j[3:, 0] = J_w.flatten()
    # If prismatic joint
    else:
        J_v = Z[i]
        J_w = zero
        j[:3, 0] = J_v.flatten()
        j[3:, 0] = J_w.flatten()
    J.append(j)

Jacobian = J[0]
for i in range(1,n):
    Jacobian = np.hstack((Jacobian,J[i]))

Jacobian = np.round(Jacobian, decimals=2)
print("\nManipulator Jacobian:")
print(Jacobian)

On = np.round(On, decimals=2)
print("\nEnd effector position:")
print(On)

q_dot = np.zeros((n,1))
print("\nGive input for q_dot for joints")
for i in range(n):
    if(joints[i]==1):
        q_dot[i] = float(input(f"Enter q_dot for Revolute joint {i + 1}: "))
    else:
        q_dot[i] = float(input(f"Enter Linear velocity for Prismatic joint {i + 1}: "))


vel = np.matmul(Jacobian,q_dot)

print("\nVelocity of end effector:")
print(vel)




 

