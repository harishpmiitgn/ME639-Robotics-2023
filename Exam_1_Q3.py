# Hrishikesh Birje
# 21110044
# Q3

import numpy as np
import math

# Function to print matrix
def print_matrix(matrix):
    for i in matrix:
        for j in i:
            print(j, end=' ')
        print()

# Using homogenous transformations
# qi are the joint variable
# di are the offsets
# ai are the link lengths

a1 = int(input("a1 = "))
a2 = int(input("a2 = "))
a3 = int(input("a3 = "))

q1 = math.degrees(float(input("q1 = ")))
q2 = math.degrees(float(input("q2 = ")))
q3 = math.degrees(float(input("q3 = ")))
q4 = math.degrees(float(input("q4 = ")))
q5 = math.degrees(float(input("q5 = ")))
q6 = math.degrees(float(input("q6 = ")))

d1 = (float(input("Offset_1 = ")))
d4 = (float(input("Offset_4 = ")))
d5 = (float(input("Offset_5 = ")))
d6 = (float(input("Offset_6 = ")))

# 1st Transformation
H01 = [[np.cos(q1), 0, np.sin(q1), 0],
       [np.sin(q1), 0, np.cos(q1), 0],
       [0, 1, 0, d1],
       [0, 0, 0, 1]]

# 2nd Transformation
H12 = [[np.cos(q2), -np.sin(q2), 0, a2*np.cos(q2)],
       [np.sin(q2), np.cos(q2), 0, a2*np.sin(q2)],
       [0, 0, 1, 0],
       [0, 0, 0, 1]]

# 3rd Transformation
H23 = [[np.cos(q3), -np.sin(q3), 0, a3*np.cos(q3)],
       [np.sin(q3), np.cos(q3), 0, a3*np.sin(q3)],
       [0, 0, 1, 0],
       [0, 0, 0, 1]]

# 4th Transformation
H34 = [[np.cos(q4), 0, np.sin(q4), 0],
       [np.sin(q4), 0, np.cos(q4), 0],
       [0, 1, 0, d4],
       [0, 0, 0, 1]]

# 5th Transformation
H45 = [[np.cos(q5), 0, -np.sin(q5), 0],
       [np.sin(q5), 0, -np.cos(q5), 0],
       [0, -1, 0, d5],
       [0, 0, 0, 1]]

# 6th Transformation
H56 = [[np.cos(q6), -np.sin(q6), 0, 0],
       [np.sin(q6), np.cos(q6), 0, 0],
       [0, 0, 1, d6],
       [0, 0, 0, 1]]

T = np.dot(H01, np.dot(H12, np.dot(H23, np.dot(H34, np.dot(H45, H56)))))


print_matrix(T)

