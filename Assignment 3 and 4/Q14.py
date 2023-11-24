
import numpy as np
n = int(input('Number of links = '))

x_dot = np.zeros((3, 1))
print("\nEnd effector linear velocity in X Y Z")
for i in range(3):
    x_dot[i, 0] = float(input(f"V[{i}]: "))

J = np.zeros((3,3))

print("\nEnter Jacobian matrix")
for i in range(3):
    for j in range(n):
        J[i,j] = float(input(f"J[{i}][{j}]: "))
print(J)

J_inverse = np.linalg.inv(J)
q_dot = np.matmul(J_inverse,x_dot)
print("\nvelocities :")
print(q_dot)