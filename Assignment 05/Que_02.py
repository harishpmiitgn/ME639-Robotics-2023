import numpy as np
import math

#We will take Jacobian Matrix and end-effector velocities as an input in the code. And after that we will apply the inverse velocity equation.
#Number of rows of Jacobian Matrix: R
R = int(input("Number of rows of Jacobian matrix: "))

#Number of columns of Jacobian Matrix: C
C = int(input("Number of column of Jacobian matrix: "))

#Taking Jacobian Matrix as input
J = []

for i in range(R):          # A for loop for row entries
    a =[]
    for j in range(C):      # A for loop for column entries
        a.append(int(input()))
    J.append(a)

#Taking end-effector velocities as input
X_dot = []
for i in range(R):          # A for loop for row entries
    b =[]
    for j in range(1):      # A for loop for column entries
        b.append(int(input()))
    X_dot.append(b)

J_inv = np.linalg.inv(J)

#Joint angular velocities calculation 
q_dot = np.dot(J_inv, X_dot)

print(q_dot)