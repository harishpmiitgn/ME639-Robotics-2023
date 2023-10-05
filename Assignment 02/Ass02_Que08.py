import numpy as np
import math

l1 = int(input("Length of link 1: "))
l2 = int(input("Length of link 2: "))
l3 = int (input("Length of link 3: "))
q1 = math.degrees(float(input("Rotation of link 1: ")))
q2 = math.degrees(float(input("Rotation of link 2: ")))

J = [[-np.sin(q1)*l1 - np.sin(q1+q2)*l2, -np.sin(q1+q2)*l2, 0],
     [np.cos(q1)*l1 + np.cos(q1+q2)*l2, np.cos(q1+q2)*l2, 0],
     [0, 0, 1],
     [0, 0, 0],
     [0, 0, 0],
     [1, 1, 0]]

print(f"The Resultant Jacobian Matrix is {J}")