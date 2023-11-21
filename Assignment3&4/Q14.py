import numpy as np

def calculate_joint_velocities(n, X_dot, J):
    J_inv = np.linalg.inv(J)
    q_dot = np.matmul(J_inv, X_dot)
    return q_dot

# Example usage:
n = int(input('No. of links = '))

X_dot = np.zeros((3, 1))
print("\nGive input of Linear velocities of end effector in x, y, z direction")
for i in range(3):
    X_dot[i, 0] = float(input(f"V[{i}]: "))

J = np.zeros((3, n))

print("\nGive input of Jacobian matrix")
for i in range(3):
    for j in range(n):
        J[i, j] = float(input(f"J[{i}][{j}]: "))
print(J)

q_dot_result = calculate_joint_velocities(n, X_dot, J)

print("\nJoint velocities are:")
print(q_dot_result)
