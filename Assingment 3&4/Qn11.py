from sympy import symbols, zeros, Matrix, diff, pprint

# Define symbolic variables
n = int(input('Enter the number of joints: '))
q = symbols(f'q1:{n + 1}')  # Joint angles
qd = symbols(f'qd1:{n + 1}')  # Joint velocities

# Initialize symbolic matrices for D, V, and tau
D = Matrix.zeros(n)
V = Matrix.zeros(n, 1)
tau = Matrix.zeros(n, 1)

# User input for the elements of the D matrix
print("Enter D(q) terms:")
for i in range(n):
    for j in range(n):
        D[i, j] = input(f'Enter element for D({i + 1}, {j + 1})(q): ')

# Calculate D(q).qdd
D_q_qdd = D * Matrix([symbols(f'qdd{i + 1}') for i in range(n)])

# User input for the potential term expression V(q)
V_expr = input('\nEnter potential term expression V(q): ')
V = Matrix([diff(V_expr, q[k]) for k in range(n)])

# Compute Christoffel symbols C(q, qd)
C = Matrix.zeros(n, 1)
for k in range(n):
    cijk = 0
    for i in range(n):
        for j in range(n):
            cijk += 0.5 * (diff(D[k, j], q[i]) + diff(D[i, k], q[j]) - diff(D[i, j], q[k])) * qd[i] * qd[j]
    C[k] = cijk

# Calculate the joint torques tau
tau = D_q_qdd + C + V

# Print the results
for k in range(n):
    print(f'Torque τ{k + 1} = ')
    pprint(tau[k])
    print('\n')

   
