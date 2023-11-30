import sympy as sp
import numpy as np
import matplotlib.pyplot as plt

def symbolic_robot_dynamics():
    # Define symbolic variables
    n = 3  # Number of joints
    q = sp.symbols(f'q1:{n+1}')
    qd = sp.symbols(f'qd1:{n+1}')
    qdd = sp.symbols(f'qdd1:{n+1}')
    m = sp.symbols(f'm1:{n+1}')
    l = sp.symbols(f'l1:{n+1}')
    g = sp.symbols('g')
    I = sp.symbols(f'I1:{n+1}')

    # Define D(q) and V(q) as symbolic expressions
    D_symbols = sp.symbols(f'd:{n}{n}')
    V_symbols = sp.symbols('V')

    # Define values for masses, lengths, and moment of inertia
    m_values = [0.5, 0.5, 0.5]
    I_values = [0.001, 0.001, 0.001]
    l_values = [0.15, 0.15, 0.15]

    # Define substitutions
    substitutions = {m[i]: m_values[i] for i in range(n)}
    substitutions.update({I[i]: I_values[i] for i in range(n)})
    substitutions.update({l[i]: l_values[i] for i in range(n)})
    g_value = 9.83
    substitutions[g] = g_value

    # Define D(q) and V(q) with substitutions
    D_elements = [
        [((m[1]*l[1]**2*sp.cos(q[1])**2)/4) + m[2]*(0.5*l[2]*sp.cos(q[1]+q[2])+l[1]*sp.cos(q[1]))**2 + I[1] + I[2] + I[3], 0, 0],
        [0, (0.25*m[1]*l[1]**2*sp.sin(q[1])**2) + m[2]*(0.25*l[2]**2 + l[1]**2 + l[1]*l[2]*sp.cos(q[2])) + I[2] + I[3], m[2]*(0.25*l[2]**2 + 0.5*l[1]*l[2]*sp.cos(q[2])) + I[3]],
        [0, m[2]*(0.25*l[2]**2 + 0.5*l[1]*l[2]*sp.cos(q[2])) + I[3], m[2]*0.25*l[2]**2 + I[3]]
    ]
    D = sp.Matrix(D_elements)

    D1 = sp.zeros(n, 1)
    for k in range(n):
        d = 0
        for i in range(n):
            d = d + (D[i, k] * qdd[i])
        D1[k] = d

    V = 0.5*m[1]*l[1]*g + m[2]*g*(l[1] + 0.5*l[2]*sp.sin(q[2])) + m[3]*g*(l[1] + l[2]*sp.sin(q[2]) + 0.5*l[3]*sp.sin(q[3]))
    phi = sp.zeros(n, 1)
    for k in range(n):
        phi[k] = sp.diff(V, q[k])

    C = sp.zeros(n, 1)
    for k in range(n):
        cijk = 0
        for i in range(n):
            for j in range(n):
                cijk = cijk + (0.5 * (sp.diff(D[k, j], q[i]) + sp.diff(D[i, k], q[j]) - sp.diff(D[i, j], q[k])) * qd[i] * qd[j])
        C[k] = cijk

    tau = sp.zeros(n, 1)
    for k in range(n):
        tau[k] = D1[k] + C[k] + phi[k]

    # Solve for qdd
    eqs = [sp.Eq(tau[i], 0.0001) for i in range(n)]
    sol = sp.solve(eqs, qdd)

    # Define numerical functions for qdd
    qdd_func = [sp.lambdify((q[0], q[1], q[2], qd[0], qd[1], qd[2]), sol[qdd[i]]) for i in range(n)]

    # Define the system of first-order ODEs
    def system(y, t):
        q1, q2, q3, qd1, qd2, qd3 = y
        qdd1_val = qdd_func[0](q1, q2, q3, qd1, qd2, qd3)
        qdd2_val = qdd_func[1](q1, q2, q3, qd1, qd2, qd3)
        qdd3_val = qdd_func[2](q1, q2, q3, qd1, qd2, qd3)

        return [qd1, qd2, qd3, qdd1_val, qdd2_val, qdd3_val]

    # Initial conditions and time points
    initial_conditions = [0, 0, 0, 0, 0, 0]
    t = np.linspace(0, 10, 100)  # Adjust the time span as needed

    # Solve the ODEs
    solution = sp.integrate.odeint(system, initial_conditions, t)

    # Extracting the results
    q1_result, q2_result, q3_result, qd1_result, qd2_result, qd3_result = solution
