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
H12 = [[np.cos(q2), -np.sin(q2), 0, link1],
       [np.sin(q2), np.cos(q2), 0, 0],
       [0, 0, 1, 0],
       [0, 0, 0, 1]]
H23 = [[1, 0, 0, link2],
       [0, 1, 0, 0],
       [0, 0, 1, d],
       [0, 0, 0, 1]]
P3 = [0, 0, link3, 1]

tran_matrix = [[0, 0, 0, 0],
          [0, 0, 0, 0],
          [0, 0, 0, 0],
          [0, 0, 0, 0]]

tran_matrix = np.dot(H01, np.dot(H12,H23))
final_value = np.dot(tran_matrix, P3)

print (f"the end-effector {final_value[0]}i + {final_value[1]}j + {final_value[2]}k")
