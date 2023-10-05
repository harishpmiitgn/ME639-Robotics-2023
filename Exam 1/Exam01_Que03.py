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

H01 = [[np.cos(q1), -np.sin(q1), 0, 0],
       [np.sin(q1), np.cos(q1), 0, 0],
       [0, 0, 1, d1],
       [0, 0, 0, 1]]

H12 = [[np.cos(q2), -np.sin(q2), 0, 0],
       [np.sin(q2), np.cos(q2), 0, -d2],
       [0, 0, 1, 0],
       [0, 0, 0, 1]]

H23 = [[np.cos(q3), -np.sin(q3), 0, 0],
       [np.sin(q3), np.cos(q3), 0, -l1],
       [0, 0, 1, 0],
       [0, 0, 0, 1]]

H34 = [[np.cos(q4), -np.sin(q4), 0, 0],
       [np.sin(q4), np.cos(q4), 0, l2],
       [0, 0, 1, -d3],
       [0, 0, 0, 1]]

H45 = [[np.cos(q5), -np.sin(q5), 0, 0],
       [0, 0, 1, 0],
       [-np.sin(q5), -np.cos(q5), 0, d4],
       [0, 0, 0, 1]]

H56 = [[np.cos(q6), -np.sin(q6), 0, 0],
       [0, 0, -1, 0],
       [np.sin(q6), np.cos(q6), 0, d5],
       [0, 0, 0, 1]]

P6 = [[0],
      [0],
      [d6],
      [1]];


# Final Transformation Matrix = H06 
H06 = np.dot(H01, np.dot(H12, np.dot(H23, np.dot(H34, np.dot(H45, H56)))))

#Final position of End-Effector
P0 = np.dot(H06, P6)

print (f"The coordinates of the end-effector is {P0[0]}i + {P0[1]}j + {P0[2]}k")

#For the values l1 = 1, l2 = 1 and all rotation angles to be 45 deg the answer we get is 
# P0 = -0.61140538 i + 2.83047663 j + 0.59316818 k 