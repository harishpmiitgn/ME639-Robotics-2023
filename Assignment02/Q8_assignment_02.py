import numpy as np
import math

link1 = int(input("link length of 1: "))
link2 = int(input("link length of 2: "))
l3 = int (input("link length of 3: "))
q1 = math.degrees(float(input("angle of link 1: ")))
q2 = math.degrees(float(input("angle of link 2: ")))

J = [[-np.sin(q1)*link1 - np.sin(q1+q2)*link2, -np.sin(q1+q2)*link2, 0],
     [np.cos(q1)*link1 + np.cos(q1+q2)*link2, np.cos(q1+q2)*link2, 0],
     [0, 0, 1],
     [0, 0, 0],
     [0, 0, 0],
     [1, 1, 0]]

print(f"Jacobian Matrix is {J}")


