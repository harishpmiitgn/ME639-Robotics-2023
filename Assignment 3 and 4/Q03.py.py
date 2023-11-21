from math import *
import numpy as np

n = int(input("no. of links = "))  

# nature of joints (R/P)
while True:
    ask = input("\n Joint information (Y/N): ").upper()
    
    if ask in ['Y', 'N']:
        break
    else:
        print("Please enter 'Y' or 'N'.")

if ask == 'Y':
    joints = []
    print("\nRevolute joint, type 'R'")
    print("Prismatic joint, type 'P'\n")
    
    for i in range(n):
        inp = input(f"Type of joint {i + 1}: ")
        if inp == 'R':
            joints.append(1)
        else:
            joints.append(0)
else:
    # values ('R') 
    joints = [1] * n


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

H = []      # homogeneous transformation matrices 
for i in range(n):
    d = DH[i][0]
    angle = DH[i][1]  
    a = DH[i][2]
    Alpha = DH[i][3]

    h = np.array([[cos(angle), -sin(angle)*cos(Alpha),  sin(angle)*sin(Alpha), a*cos(angle)],
                  [sin(angle),  cos(angle)*cos(Alpha), -cos(angle)*sin(Alpha), a*sin(angle)],
                  [0         ,  sin(Alpha)           ,  cos(Alpha)           , d           ],
                  [0         ,  0                    ,  0                    , 1           ]])
    h = np.round(h, decimals=2)
    H.append(h)


H0 = []        # homogeneous transformation matrices 
H0.append(H[0])
h = H[0]
for i in range(1,n):
    h = np.matmul(h,H[i])
    H0.append(h)


R = []          # Rotation matrices 
O = []          # distance of end point 

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


On = O[n]
J=[]        # columns of jacobian matrix for ith link
for i in range(n):
    j = np.zeros((6,1))
   
    if(joints[i]==1):
        J_v = np.cross(Z[i].flatten(), (On - O[i]).flatten())
        J_w = Z[i]
        j[:3, 0] = J_v
        j[3:, 0] = J_w.flatten()

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
print("\nJacobian:")
print(Jacobian)

On = np.round(On, decimals=2)
print("\nend effector coordinate:")
print(On)

q_dot = np.zeros((n,1))
print("\q_dot for joints")
for i in range(n):
    if(joints[i]==1):
        q_dot[i] = float(input(f"q_dot for Revolute joint {i + 1}: "))
    else:
        q_dot[i] = float(input(f"Linear velocity for Prismatic joint {i + 1}: "))


vel = np.matmul(Jacobian,q_dot)

print("\nVelocity :")
print(vel)




 

