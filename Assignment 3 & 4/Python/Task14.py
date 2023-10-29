# joint velocities using end-effector cartesian velocities


import numpy as np
n = int(input('No. of links = '))

X_dot = np.zeros((3, 1))
print("\nGive input of Linear velocities of end effector in x,y,z direction")
for i in range(3):
    X_dot[i, 0] = float(input(f"V[{i}]: "))

#print(X_dot)
J = np.zeros((3,3))

print("\nGive input of Jacobian matrix")
for i in range(3):
    for j in range(n):
        J[i,j] = float(input(f"J[{i}][{j}]: "))
print(J)

J_inv = np.linalg.inv(J)
q_dot = np.matmul(J_inv,X_dot)
print("\nJoint velocities are:")
print(q_dot)