#Name:- Barde Vedant
#Roll Number:- 21110043

import numpy as np
import math

# I am defining all the offset values to be 1 unit for the simplicity of calculation
d1 = int(1);
d2 = int(1);
d3 = int(1);
d4 = int(1);
d5 = int(1);
d6 = int(1);

#Taking input for the length of link
l1 = int(input("Length of Link 1: "))
l2 = int(input("Length of Link 2: "))

#Taking input of joint rotations
q1 = math.degrees(float(input("Rotation 1: ")))
q2 = math.degrees(float(input("Rotation 2: ")))
q3 = math.degrees(float(input("Rotation 3: ")))
q4 = math.degrees(float(input("Rotation 4: ")))
q5 = math.degrees(float(input("Rotation 5: ")))
q6 = math.degrees(float(input("Rotation 6: ")))


R01 = [[np.cos(q1), -np.sin(q1), 0],
       [np.sin(q1), np.cos(q1), 0],
       [0, 0, 1]]

R12 = [[np.cos(q2), -np.sin(q2), 0],
       [np.sin(q2), np.cos(q2), 0],
       [0, 0, 1]]

R23 = [[np.cos(q3), -np.sin(q3), 0],
       [np.sin(q3), np.cos(q3), 0],
       [0, 0, 1]]

R34 = [[np.cos(q4), -np.sin(q4), 0],
       [np.sin(q4), np.cos(q4), 0],
       [0, 0, 1]]

R45 = [[np.cos(q5), -np.sin(q5), 0],
       [0, 0, 1],
       [-np.sin(q5), -np.cos(q5), 0]]

R56 = [[np.cos(q6), -np.sin(q6), 0],
       [0, 0, -1],
       [np.sin(q6), np.cos(q6), 0]]

d01 = [[0],
      [0],
      [d1]];

d12 = [[0],
      [-d2],
      [0]];

d23 = [[0],
      [l1],
      [0]];

d34 = [[0],
      [l2],
      [-d3]];

d45 = [[0],
      [0],
      [d4]];

d56 = [[0],
      [0],
      [d5]];

d02 = d01 + np.dot(R01, d12)
d03 = d02 + np.dot(R01, np.dot(R12, d23))
d04 = d03 + np.dot(R01, np.dot(R12, np.dot(R23, d34)))
d05 = d04 + np.dot(R01, np.dot(R12, np.dot(R23, np.dot(R34, d45))))
d06 = d05 + np.dot(R01, np.dot(R12, np.dot(R23, np.dot(R34, np.dot(R45, d56)))))
