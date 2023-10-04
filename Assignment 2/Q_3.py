import numpy as np
import math

print('ai are link lengths')
print('qi are angles')

a1 = int(input("a1 = "))
a2 = int(input("a2 = "))

d = float(input("d = "))

q1 = math.degrees(float(input("q1 = ")))
q2 = math.degrees(float(input("q2 = ")))

H01 = [[np.cos(q1), -np.sin(q1), 0, a1*np.cos(q1)],
       [np.sin(q1), np.cos(q1), 0, a1*np.sin(q1)],
       [0, 0, 1, 0],
       [0, 0, 0, 1]]
H12 = [[np.cos(q2), np.sin(q2), 0, a2*np.cos(q2)],
       [np.sin(q2), -np.cos(q2), 0, a2*np.sin(q2)],
       [0, 0, 1, 0],
       [0, 0, 0, 1]]
H23 = [[1, 0, 0, 0],
       [0, 1, 0, 0],
       [0, 0, 1, d],
       [0, 0, 0, 1]]
P3 = [0, 0, d, 1]

T = [[0, 0, 0, 0],
     [0, 0, 0, 0],
     [0, 0, 0, 0],
     [0, 0, 0, 0]]

T = np.dot(H01, np.dot(H12, H23))
ans = np.dot(T, P3)

print (f"the end-effector {ans[0]}i + {ans[1]}j + {ans[2]}k")
