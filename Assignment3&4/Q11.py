from sympy import symbols, Matrix, diff, pprint, sin, cos

def calculate_torque(D, qdd, q, n, m, g, V):
    # Calculate D(q).qdd
    D1 = D * Matrix(qdd)

    # Calculate potential terms
    phi = Matrix([diff(V, qi) for qi in q])

    # Calculate Christoffel symbols
    C = Matrix([sum(0.5 * (diff(D[k, j], q[i]) + diff(D[i, k], q[j]) - diff(D[i, j], q[k])) * qd[i] * qd[j]
                   for i in range(n) for j in range(n)) for k in range(n)])

    # Calculate τ (torque)
    tau = D1 + C + phi

    return tau

def main():
    # Define symbolic variables and parameters
    n = 2
    q = symbols('q1:{}'.format(n+1))
    qd = symbols('qd1:{}'.format(n+1))
    qdd = symbols('qdd1:{}'.format(n+1))
    m = symbols('m1:{}'.format(n+1))
    l = symbols('l1:{}'.format(n+1))
    g = symbols('g')
    I = symbols('I1:{}'.format(n+1))

    # Predefined values
    D_elements = [[0.25*m[0]*l[0]**2 + m[1]*l[0]**2 + I[0], 0.5*m[1]*l[0]*l[1]*cos(q[1] - q[0])],
                  [0.5*m[1]*l[0]*l[1]*cos(q[1] - q[0]), 0.25*m[1]*l[1]**2 + I[1]]]

    D = Matrix(D_elements)

    V = 0.5*m[0]*g*l[0]*sin(q[0]) + m[1]*g*l[0]*sin(q[0]) + 0.5*m[1]*g*l[1]*sin(q[1])

    # Calculate torque
    tau = calculate_torque(D, qdd, q, n, m, g, V)

    # Display results
    for k in range(n):
        print(f'Torque τ{k + 1} = ')
        pprint(tau[k])
        print('\n')

if __name__ == "__main__":
    main()
