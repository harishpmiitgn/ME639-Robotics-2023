import numpy as np
import math

print('ai are link lengths')
print('qi are angles')

a1 = int(input("a1 = "))
a2 = int(input("a2 = "))
a3 = int(input("a3 = "))

q1 = math.degrees(float(input("q1 = ")))
q2 = math.degrees(float(input("q2 = ")))
q3 = math.degrees(float(input("q3 = ")))


J=[[-a1*np.sin(q1) - a2*np.sin(q1+q2) - a3*np.sin(q1+q2+q3), -a3*np.sin(q1+q2+q3) - a2*np.sin(q1+q2), -a3*np.sin(q1+q2+q3)],
   [a3*np.cos(q1+q2+q3) + a2*np.cos(q1+q2) + a1*np.cos(q1), a3*np.cos(q1+q2+q3) + a2*np.cos(q1+q2), a3*np.cos(q1+q2+q3)],
   [0, 0, 0],
   [0, 0, 0],
   [0, 0, 0],
   [1, 1, 1]]


print(f"Jacobian Matrix is {J}")