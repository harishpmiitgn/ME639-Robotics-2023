import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

# Define robot parameters
m1, m2, m3 = 1.0, 1.0, 1.0  
lc2, lc3 = 0.5, 0.5  
l2, l3 = 1.0, 1.0 
g = 9.8 

# Define initial conditions
initial_q = np.array([2, -1, 1])  # Initial joint angles
initial_qdott = np.array([0.0, 0.0, 0.0])  # Initial joint velocities
initial_qdott_motor = np.array([0.0, 0.0, 0.0])  # Initial motor velocities
initial_q_I_motor = np.array([0.0, 0.0, 0.0])  # Initial motor currents

# Desired joint angles and velocities
q_desired = np.array([2, 1, 4])
qdott_desired = np.array([0, 0, 0])
tau_motor_desired = np.array([0.00001, 0.00011, 0.0002]) # Small constant motor torques


# Initialize state variables
q = initial_q.copy()
qdott = initial_qdott.copy()
qdott_motor = initial_qdott_motor.copy()
Imotor = initial_q_I_motor.copy()


tmax = 20.0
numsteps = 10000
dt = tmax / numsteps
time_values = np.linspace(0, tmax, numsteps)


# Motor dynamics
Jm = np.array([1, 1, 1])  #  Inertias of motor
Rm = np.array([.01, 0.5, 0.5])  #  resistances
Lm = np.array([0.01, 0.05, 0.15])  # inductances
Kb = np.array([0, 0, 0])  # back-emf constants
Bm = np.array([0, 0, 0.5])  #  damping coefficients
Kt = np.array([0.1, 0.1, 0.1])  # torque constants
\
def dynamics(state, t):
    q1, q2, q3, q1_dot, q2_dot, q3_dot, q1_dot_motor, q2_dot_motor, q3_dot_motor, I1, I2, I3 = state
    q = np.array([q1, q2, q3])
    qdott = np.array([q1_dot, q2_dot, q3_dot])
    qdott_motor = np.array([q1_dot_motor, q2_dot_motor, q3_dot_motor])
    q_ddot_motor = np.zeros(3)
    Imotor = np.array([I1, I2, I3])
    I_dot_motor = np.zeros(3)
    tau_motor = np.zeros(3)
    V_back = np.zeros(3)
    V_motor = np.zeros(3)
   
    C2 = np.cos(q2)
    S2 = np.sin(q2)
    C3 = np.cos(q3)
    S3 = np.sin(q3)
    C23 = np.cos(q2 + q3)
    S23 = np.sin(q2 + q3)

    M = np.array([
        [(1/2 * m1 * lc2**2 + 1/4 * m2 * l2**2 * C2**2 + 1/4 * m3 * l3**2 * C23**2 + m3 * l2**2 * C2**2 + m2 * l2 * l3 * C2 * C23), 0, 0],
        [0, (1/4 * m2 * l2**2 + m3 * (l2**2 + lc3**2 + l2 * l3 * C3)), 0],
        [0, 0, (1/3 * m3 * l3**2)]
    ])

    C = np.array([
        [0, (-1/4 * m2 * l2**2 * S2 - m3 * l2**2 * S2 - m3 * l2 * l3 * S23 - m3 * lc2**2 * S23), (-m3 * l2 * l3 * C2 * S23 - m3 * lc3**2 * S23)],
        [0, 0, -m2 * l2 * l3 * S3],
        [0, 0, 0]
    ])

    G = np.array([
        lc2 * m2 * g * C2 + l2 * m3 * g * C2 + lc3 * m2 * g * C23,
        0,
        m3 * g * lc3 * C23
    ])
   
    # Motor Dynamics
    V_back = Kb * qdott_motor
    Imotor = tau_motor_desired / (Kt * Rm)
    q_ddot_motor = (tau_motor_desired - Bm * qdott_motor - Kt * Imotor) / Jm
    qdott_motor += q_ddot_motor * dt 
    V_motor = Imotor * Rm
    I_dot_motor = (V_motor - Rm * Imotor - V_back) / Lm
    tau_motor = V_motor * Kt

    q_ddot = np.linalg.solve(M, tau_motor - np.dot(C, qdott) - G)

    return [qdott[0], qdott[1], qdott[2], q_ddot[0], q_ddot[1], q_ddot[2], q_ddot_motor[0], q_ddot_motor[1], q_ddot_motor[2], I_dot_motor[0], I_dot_motor[1], I_dot_motor[2]]


# Initial conditions for the ODE solver
stateini = np.concatenate((q, qdott, qdott_motor, Imotor))

# Use odeint to integrate
solution = odeint(dynamics, stateini, time_values)


q1_values = solution[:, 0]
q2_values = solution[:, 1]
q3_values = solution[:, 2]


plt.figure()
plt.suptitle(' Joint Angles (q) vs. Time (t) ')

plt.subplot(321)
plt.plot(time_values, q1_values, label='q1')
plt.xlabel('t (s)')
plt.ylabel('q1 (radian)')
plt.grid()
plt.legend()

plt.subplot(322)
plt.plot(time_values, q2_values, label='q2')
plt.xlabel('t (s)')
plt.ylabel('q2 (radian)')
plt.grid()
plt.legend()

plt.subplot(323)
plt.plot(time_values, q3_values, label='q3')
plt.xlabel('t (s)')
plt.ylabel('q3 (radian)')
plt.grid()
plt.legend()

plt.show()