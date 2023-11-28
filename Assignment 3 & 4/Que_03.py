import numpy as np

n = int(input("Number of Joints: "))
print("Given the configuration of the manipulator below.\nFor P, put 0 and for R, put 1 in one line with space.")
Configuration = list(map(int, input("Enter the configuration here: ").split()))

if len(Configuration) != n:
    print("Number of configurations does not match the number of joints. Please provide the correct input.")
    exit()

Rows = n
Columns = 4

# Initialize the DH_Matrix
DH_Matrix = []
print("Give the entries row-wise:")

# For user input
for _ in range(Rows):
    r = []
    for __ in range(Columns):
        r.append(float(input()))
    DH_Matrix.append(r)

# Print the DH Matrix given by the user
print("Your DH Matrix is as below:")
for _ in range(Rows):
    for __ in range(Columns):
        print(DH_Matrix[_][__], end=" ")
    print()

H0i = []
d0i = []
R0i = []

# Calculating the Homogeneous Transformation Matrices from DH Table
for i in range(0, n):
    Hi = [[np.cos(DH_Matrix[i][1]), -np.sin(DH_Matrix[i][1]) * np.cos(DH_Matrix[i][3]),
           np.sin(DH_Matrix[i][1]) * np.sin(DH_Matrix[i][3]), DH_Matrix[i][2] * np.cos(DH_Matrix[i][1])],
          [np.sin(DH_Matrix[i][1]), np.cos(DH_Matrix[i][1]) * np.cos(DH_Matrix[i][3]),
           -np.cos(DH_Matrix[i][1]) * np.sin(DH_Matrix[i][3]), DH_Matrix[i][2] * np.sin(DH_Matrix[i][1])],
          [0, np.sin(DH_Matrix[i][3]), np.cos(DH_Matrix[i][3]), DH_Matrix[i][0]],
          [0, 0, 0, 1]]
    H0i.append(Hi)

    di = [[DH_Matrix[i][2] * np.cos(DH_Matrix[i][1])],
          [DH_Matrix[i][2] * np.sin(DH_Matrix[i][1])],
          [DH_Matrix[i][0]]]
    d0i.append(di)

    Ri = [[np.cos(DH_Matrix[i][1]), -np.sin(DH_Matrix[i][1]) * np.cos(DH_Matrix[i][3]),
           np.sin(DH_Matrix[i][1]) * np.sin(DH_Matrix[i][3])],
          [np.sin(DH_Matrix[i][1]), np.cos(DH_Matrix[i][1]) * np.cos(DH_Matrix[i][3]),
           -np.cos(DH_Matrix[i][1]) * np.sin(DH_Matrix[i][3])],
          [0, np.sin(DH_Matrix[i][3]), np.cos(DH_Matrix[i][3])]]
    R0i.append(Ri)

# print(H0i)
# print(d0i)
# print(R0i)

def R0n(n):
    if n == 1:
        R0n_final = R0i[0]

    if n != 1:
        R0n_final = np.dot(R0i[0], R0i[1])
        for k in range(n - 2):
            R0n_final = np.dot(R0n_final, R0i[k + 2])
    return R0n_final

def H0n(n):
    if n == 1:
        H0n_final = H0i[0]

    if n != 1:
        H0n_final = np.dot(H0i[0], H0i[1])
        for j in range(n - 2):
            H0n_final = np.dot(H0n_final, H0i[j + 2])
    
    return H0n_final

def d0n(n):
    if n == 1:
        d0n_final = d0i[0]

    if n != 1:
        d0n_final = np.dot(d0i[0], d0i[1])
        for l in range(n - 2):
            d0n_final = np.dot(d0n_final, d0i[l + 2])
    return d0n_final
# print(R0n(n))
# print(H0n(n))
# print(d0n(n))

# b) Calculating the end-Effector position
dnn = [[0],
       [0],
       [0],
       [1]]  # End effector coordinate with respect to the last current frame that is n itself.
P0n = np.dot(H0n(n), dnn)
print(f"The end effector position is {P0n[0, 0]} i + {P0n[1, 0]} j + {P0n[2, 0]} k")

# a) Calculating the Jacobian matrix
Jw = np.zeros((6, n))
Jv = np.zeros((6, n))

k_cap = np.array([[0], [0], [1]])

# for i in range(n):
#     if Configuration[i] == 0:
#         Jw[:3, i:i + 1] = np.zeros((3, 1))
#         Jv[:3, i:i + 1] = np.dot(R0n(i), k_cap)
#     else:
#         Jw[:3, i:i + 1] = np.dot(R0n(i), k_cap)
#         Jv[:3, i:i + 1] = np.dot(R0n(i), (d0n(n) - d0n(i)))

# # c) Calculating the End-Effector Velocities
# print("Insert the values of q1_dot, q2_dot, ...... below")
# q_dot = np.array(list(map(float, input().split())))
# X_dot = np.dot(np.hstack((Jv, Jw)), q_dot)

# print("End-Effector Velocities:")
# print(X_dot)
