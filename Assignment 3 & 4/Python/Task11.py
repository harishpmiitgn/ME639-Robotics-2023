from sympy import zeros,symbols, Matrix, diff, pprint

# Define symbolic variables
n = int(input('Enter no. of joints = '))  # Number of joints or degrees of freedom
q = symbols('q1:{}'.format(n+1))  # q (joint angles)
qd = symbols('qd1:{}'.format(n+1))  # qdot (joint velocities)
qdd = symbols('qdd1:{}'.format(n+1))  # qddot (joint accelerations)
m = symbols('m1:{}'.format(n+1))  # link masses
l = symbols('l1:{}'.format(n+1)) # link lengths
g = symbols('g')  # gravitational acceleration
I = symbols('I1:{}'.format(n+1))  # inertial terms

# Define D(q) and V(q) as symbolic expressions
# Define symbolic variables
D_symbols = symbols('d:{}{}'.format(n, n))
V_symbols = symbols('V')

# Take user input for the elements of the matrix D
print("Enter D(q) terms")
D_elements = [[(input(f'Enter element for d{i+1}{j+1}: ')) for j in range(n)] for i in range(n)]
# Create the matrix D
D = Matrix(D_elements)
# Display the matrix D
# print("\nMatrix D(q):")
# pprint(D)
D1 = Matrix(zeros(n, 1))
for k in range(n):
    d=0
    for i in range(n):
        d = d + (D[i,k]*qdd[i])
    D1[k] = d
# print("\nMatrix D(q).qdd:")
# pprint(D1)

V = input('\nEnter potential term expression V(q) = ')
# potential terms
phi = Matrix(zeros(n, 1))
for k in range(n):
    phi[k] = diff(V, q[k])
# print("\nMatrix Φ_k:")
# pprint(phi)


# Christoffel symbols
C =  Matrix(zeros(n, 1))
for k in range(n):
    cijk = 0
    for i in range(n):
        for j in range(n):
            cijk = cijk +  (0.5 * (diff(D[k,j],q[i]) + diff(D[i,k],q[j]) - diff(D[i,j],q[k]))*qd[i]*qd[j])
    C[k] = cijk
# print("\nChristoffel symbol Matrix C(q,qd).qd:")
# pprint(C)

tau = Matrix(zeros(n, 1))
for k in range(n):
    tau[k] = D1[k] + C[k] + phi[k]


for k in range(n):
    print(f'τ{k+1} = ')
    pprint(tau[k])
    print('\n')




# For 2R manipulator

# d11 =  0.25*m1*l1**2+m2*l1**2+I1
# d12 =  0.5*m2*l1*l2*cos(q2-q1)
# d21 =  0.5*m2*l1*l2*cos(q2-q1)
# d22 =  0.25*m2*l2**2+I2

# V(q) =  0.5*m1*g*l1*sin(q1)+m2*g*l1*sin(q1)+0.5*m2*g*l2*sin(q2)