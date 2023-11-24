import numpy as np
import math


r = int(input(" Rows, Jacobian matrix"))


c = int(input("Column, Jacobian matrix"))


J = []             # Jacobian Matrix 

for i in range(r):          # rows
    a =[]
    for j in range(r):      # columns
        a.append(int(input()))
    J.append(a)


x_dot = []                  #end-effector velocities 
for i in range(r):          # rows
    b =[]
    for j in range(1):      # columns
        b.append(int(input()))
    x_dot.append(b)

J_inv = np.linalg.inv(J)


q_dot = np.dot(J_inv, x_dot)

print(q_dot)