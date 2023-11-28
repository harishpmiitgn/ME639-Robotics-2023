import numpy as np
import math
#Taking Rotation Matrix as input
R = [[1,2,3],[1,4,5],[5,6,9]]

# for i in range(3):          # A for loop for row entries
#     a =[]
#     for j in range(3):      # A for loop for column entries
#         a.append(int(input()))
#     R.append(a)

#Taking Rotation Matrix for first 3 links as input
R03 = [[1,2,3],[12,5,3],[1,58,7]]

# for p in range(3):          # A for loop for row entries
#     b =[]
#     for q in range(3):      # A for loop for column entries
#         b.append(int(input()))
#     R03.append(a)

#Joint angles of the Spherical Wrist
q = [0, 0, 0]

#Transpose function 
def transpose(A, B):
    for i in range(2):
        for j in range(2):
            B[i][j] = A[j][i]

#Finding the R36 matrix (Rotation matrix of the wrist)
R03_transpose = []
R03_transpose = transpose(R03, R03_transpose)
print(R03_transpose)

R36  = np.dot(R03_transpose, R)
# R36 = [[sum(a * b for a, b in zip(A_row, B_col)) 
#                         for B_col in zip(*R)]
#                                 for A_row in R03_transpose]
# for r in R36:
#     print(r)

#Calculating the Joint angles using inverse kinematics
if (R36[1][3] ==0 & R36[2][3] == 0):
    if(R36[3][3] == 1):
        q[1] = 0
        q[0] = 0
        q[2] = math.atan(R36[2][1], R36[1][1])
    elif(R36[3][3] == -1):
        q[1] = 3.14
        q[0] = 0
        q[2] = math.atan(-R36[2][1], -R36[2][2])

else:
    q[1] = math.atan(R36[3][3], math.sqrt(1 - R36[3][3]**2))
    q[0] = math.atan( R36[1][3], R36[2][3])
    q[2] = math.atan(-R36[3][1], R36[3][2])

#Printing the values of joint angles
print("q[0] = theta_4 \nq[1] = theta_5 \nq[2] = theta_6")
print(q)