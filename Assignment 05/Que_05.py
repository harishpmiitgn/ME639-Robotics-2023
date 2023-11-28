import numpy as np
import math
#Taking Rotation Matrix as input
R = []

for i in range(3):          # A for loop for row entries
    a =[]
    for j in range(3):      # A for loop for column entries
        a.append(int(input()))
    R.append(a)

#Taking Rotation Matrix for first 3 links as input
R03 = []

for p in range(3):          # A for loop for row entries
    b =[]
    for q in range(3):      # A for loop for column entries
        b.append(int(input()))
    R03.append(a)

#Joint angles of the Spherical Wrist
q = [0, 0, 0]

#Transpose function 
def transpose(A, B):
    for i in range(3):
        for j in range(3):
            B[i][j] = A[j][i]

#Finding the R36 matrix (Rotation matrix of the wrist)
R03_transpose = R03[:][:]
R03_transpose = transpose(R03, R03_transpose)

R36  = np.dot(R03_transpose, R)

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