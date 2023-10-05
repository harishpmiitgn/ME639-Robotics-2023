import numpy as np
import math

l1 = int(input("Length of link 1: "))
l2 = int(input("Length of link 2: "))
l3 = int (input("Length of link 3: "))
q1 = math.degrees(float(input("Rotation of link 1: ")))
q2 = math.degrees(float(input("Rotation of link 2: ")))
d = (float(input("Error adjustment of link 3: ")))
H01 = [[np.cos(q1), -np.sin(q1), 0, 0],
       [np.sin(q1), np.cos(q1), 0, 0],
       [0, 0, 1, 0],
       [0, 0, 0, 1]]
H12 = [[np.cos(q2), -np.sin(q2), 0, l1],
       [np.sin(q2), np.cos(q2), 0, 0],
       [0, 0, 1, 0],
       [0, 0, 0, 1]]
H23 = [[1, 0, 0, l2],
       [0, 1, 0, 0],
       [0, 0, 1, d],
       [0, 0, 0, 1]]
P3 = [0, 0, l3, 1]

transformation_matrix = [[0, 0, 0, 0],
          [0, 0, 0, 0],
          [0, 0, 0, 0],
          [0, 0, 0, 0]]

transformation_matrix = np.dot(H01, np.dot(H12,H23))
answer = np.dot(transformation_matrix, P3)

print (f"The coordinates of the end-effector is {answer[0]}i + {answer[1]}j + {answer[2]}k")