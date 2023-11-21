import numpy as py
import math

J_inv = np.linalg.pinv(J)
q_dot = np.dot(J_inv, X_dot)

print(f"Joint velocities = {q_dot}")