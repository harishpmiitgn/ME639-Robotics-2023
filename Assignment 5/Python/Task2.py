# joint velocities using end-effector cartesian velocities


import numpy as np
n = int(input('No. of links = '))

X_dot = np.zeros((6, 1))
print("\nGive input of Linear velocities of end effector in x,y,z direction")

X_dot[0, 0] = float(input("V_x : "))
X_dot[1, 0] = float(input("V_y : "))
X_dot[2, 0] = float(input("V_z : "))
X_dot[3, 0] = float(input("w_x : "))
X_dot[4, 0] = float(input("w_y : "))
X_dot[5, 0] = float(input("w_z : "))

#print(X_dot)
J = np.zeros((6,n))

print("\nGive input of Jacobian matrix of Dimension 6 * n")
print("If there is no value then give 0")
for i in range(6):
    for j in range(n):
        J[i,j] = float(input(f"J[{i}][{j}]: "))
#print(J)

if n!=6:
    J_trans = J.T
    a = np.matmul(J_trans,J)
    b = np.linalg.inv(a)
    c = np.matmul(b,J_trans)
    q_dot = np.matmul(c,X_dot)
else:
    J_inv = np.linalg.inv(J)
    q_dot = np.matmul(J_inv,X_dot)
print("\nJoint velocities are:")
print(q_dot)