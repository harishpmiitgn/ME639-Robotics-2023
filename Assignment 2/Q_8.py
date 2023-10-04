import numpy as np
import math

print('ai are link lengths')
print('qi are angles')

a1 = int(input("a1 = "))
a2 = int(input("a2 = "))

d = float(input("d = "))

q1 = math.degrees(float(input("q1 = ")))
q2 = math.degrees(float(input("q2 = ")))

J = [[-a1*np.sin(q1) - a2*np.sin(q1 + q2), -a2*np.sin(q1 + q2), 0],
     [a1*np.cos(q1) + a2*np.sin(q1 + q2), a2*np.cos(q1 + q2), 0],
     [0, 0, -1],
     [0, 0, 0],
     [0, 0, 0],
     [1, 1, 0]]

print(f"Jacobian Matrix is {J}")