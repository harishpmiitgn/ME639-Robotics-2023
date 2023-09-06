import numpy as np
import math

link1 = int(input("link Length 1: "))
link2 = int(input("link Length 2: "))
link3 = int (input("link Length 3: "))
link4 = int(input("link Length 4: "))
link5 = int(input("link Length 5: "))
link6 = int(input("link Length 6: "))
q1 = math.degrees(float(input("angle of link 1: ")))
q2 = math.degrees(float(input("angle of link 2: ")))
q3 = math.degrees(float(input("angle of link 3: ")))
q4 = math.degrees(float(input("angle of link 4: ")))
q5 = math.degrees(float(input("angle of link 5: ")))
q6 = math.degrees(float(input("angle of link 6: ")))
d = (float(input("Error for link 3: ")))
H01 = [[-np.sin(q1)*np.sin(q2), -np.sin(q1)*np.cos(q2), np.cos(q1), -np.sin(q1)*link1],
       [np.cos(q1)*np.sin(q2), np.cos(q1)*np.cos(q2), np.sin(q1), np.cos(q1)*link1],
       [-np.cos(q2), np.sin(q2), 0, 0],
       [0, 0, 0, 0]]
H12 = [[np.cos(q3), -np.sin(q3), 0, 0],
       [np.sin(q3), np.cos(q3), 0, link2],
       [0, 0, 1, 0],
       [0, 0, 0, 1]]
H23 = [[np.cos(q4), -np.sin(q4), 0, 0],
       [np.sin(q4), np.cos(q4), 0, link3],
       [0, 0, 1, 0],
       [0, 0, 0, 1]]
       
H34 = [[np.cos(q5), -np.sin(q5), 0, 0],
       [0, 0, -1, link4],
       [np.sin(q5), np.cos(q5), 0, 0],
       [0, 0, 0, 1]]
       
H45 = [[0, 0, 1, 0],
       [np.sin(q6), np.cos(q6), 0, 0],
       [-np.cos(q6), np.sin(q6), 0, link5],
       [0, 0, 0, 1]]

       
P6 = [0, link6, 0 , 1]

tran_matrix = [[0, 0, 0, 0],
          [0, 0, 0, 0],
          [0, 0, 0, 0],
          [0, 0, 0, 0]]

tran_matrix = np.dot(H01, np.dot(H12,H23,H34,H45))
final_value = np.dot(tran_matrix, P6)

print (f"the end-effector {final_value[0]}i + {final_value[1]}j + {final_value[2]}k")