import sympy as sp

def equations_of_motion(D, potential_energy, q, q_dot):
    n = len(q)

    t = sp.symbols('t')

    q_t = [sp.Function('q{}'.format(i + 1))(t) for i in range(n)]    # q as a function of time

    q_dot_t = [sp.diff(qi, t) for qi in q_t]    # q_dot as derivative of q with respect to time

    T = 0.5 * sp.Matrix(q_dot_t).T.dot(D.subs({q[i]: q_t[i] for i in range(n)}).dot(sp.Matrix(q_dot_t)))

    L = T - potential_energy.subs({q[i]: q_t[i] for i in range(n)})

    tau = [sp.diff(sp.diff(L, q_dot_i), t) - sp.diff(L, q_i) for q_i, q_dot_i in zip(q_t, q_dot_t)]

    return tau

