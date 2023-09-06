import numpy as np
import math

link1 = int(input("link Length 1: "))
link2 = int(input("link Length 2: "))
link3 = int (input("link Length 3: "))
q1 = math.degrees(float(input("angle of link 1: ")))
q2 = math.degrees(float(input("angle of link 2: ")))
d = (float(input("Error for link 3: ")))
H01 = [[np.cos(q1), -np.sin(q1), 0, 0],
       [np.sin(q1), np.cos(q1), 0, 0],
       [0, 0, 1, 0],
       [0, 0, 0, 1]]
Rx90 = [[1, 0, 0],
        [0, 0, -1],
        [0, 1, 0]]
RZq2 = [[np.cos(q2), -np.sin(q2), 0],
        [np.sin(q2), np.cos(q2), 0],
        [0, 0, 1]]
R12 = np.dot(Rx90, RZq2)

H12 = [[R12[0, 0], R12[0, 1], R12[0, 2], 0],
       [R12[1, 0], R12[1, 1], R12[1, 2], 0],
       [R12[2, 0], R12[2, 1], R12[2, 2], link1],
       [0, 0, 0, 1]]
H23 = [[1, 0, 0, link2+d],
       [0, 1, 0, 0],
       [0, 0, 1, 0],
       [0, 0, 0, 1]]
P3 = [0, -link3, 0, 1]

transformation_matrix = [[0, 0, 0, 0],
          [0, 0, 0, 0],
          [0, 0, 0, 0],
          [0, 0, 0, 0]]

transformation_matrix = np.dot(H01, np.dot(H12,H23))
final_vaue = np.dot(transformation_matrix, P3)

print (f"end-effector {final_vaue[0]}i + {final_vaue[1]}j + {final_vaue[2]}k")
