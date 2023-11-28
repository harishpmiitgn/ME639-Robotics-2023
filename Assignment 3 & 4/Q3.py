import numpy as np

def transformation_matrix(alpha, a, d, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

n = input('No. of links: ')

# DH parameters
dh = np.zeros((n, 4))
for i in range(n):  # A for loop for row entries
    a = []
    for j in range(4):  # A for loop for column entries
        a.append(int(input()))
    dh.append(a)

#Tranformation matrix
T = np.eye(4)
for i in range(num_links):
    alpha, a, d, theta = dh[i]
    T_i = transformation_matrix(alpha, a, d, theta)
    T = np.matmul(T, T_i)





J = np.zeros((6, n))
