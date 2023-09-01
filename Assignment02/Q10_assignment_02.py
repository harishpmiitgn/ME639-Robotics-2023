import numpy as np
import math

link1 = int(input("Length of link 1: "))
link2 = int(input("Length of link 2: "))
link3 = int (input("Length of link 3: "))
q1 = math.degrees(float(input("angle of link 1: ")))
q2 = math.degrees(float(input("angle of link 2: ")))
q3 = math.degrees(float(input("angle of link 3: ")))

J = [[-np.sin(q1+q2+q3)*link3 - np.sin(q1+q2)*link2 - np.sin(q1)*link1, -np.sin(q1+q2+q3)*link3 - np.sin(q1+q2)*link2, -np.sin(q1+q2+q3)*link3],
    [np.cos(q1+q2+q3)*link3 + np.cos(q1+q2)*link2 + np.cos(q1)*link1, np.cos(q1+q2+q3)*link3 + np.cos(q1+q2)*link2, np.cos(q1+q2+q3)*link3],
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
    [1, 1, 1]]

print(f"The Jacobian Matrix is {J}")
