import numpy as np
import matplotlib.pyplot as plt
from sympy import zeros,symbols, Matrix, diff, pprint, lambdify, Eq, solve, dsolve,sin, cos
from prettytable import PrettyTable


print("\n3 Independent joint Control\n")
# Create a table
table = PrettyTable()
# Define the columns
table.field_names = ["Control", "Number code"]

# Add data rows
table.add_row(["Simple", "1"])
table.add_row(["Feed Forward", "2"])
table.add_row(["Computed Torque", "3"])
print(table)

inp = input("\nEnter the number code for choosing joint control: ")



# Time vector
t = np.linspace(0, 10, 100)  # Adjust the time span and resolution as needed
# Desired joint angle trajectories as functions of time
q1_desired = np.sin(0.5 * t)
q2_desired = 0.5 * np.cos(t)
q3_desired = 0.2 * t

disturbance = np.array([0.0057745 , 0.01754855, 0.0015653 ])

# PD controller gains
kp = 1.0  # Proportional gain
kd = 0.1  # Derivative gain
kf = 0.1  # Feed forward constant


if inp =='1':
    # Joint angles and velocities
    q2 = np.zeros_like(t)
    q1 = np.zeros_like(t)
    q3 = np.zeros_like(t)
    qd1 = np.zeros_like(t)
    qd2 = np.zeros_like(t)
    qd3 = np.zeros_like(t)

    prev_error1 = 0
    prev_error2 = 0
    prev_error3 = 0

    # PD control loop
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        error1 = q1_desired[i] - q1[i - 1]
        error2 = q2_desired[i] - q2[i - 1]
        error3 = q3_desired[i] - q3[i - 1]
    
        # Proportional and Derivative terms
        P1 = kp * error1
        P2 = kp * error2
        P3 = kp * error3

        D1 = kd * (error1 - prev_error1) / dt
        D2 = kd * (error2 - prev_error2) / dt
        D3 = kd * (error3 - prev_error3) / dt

        # PD control law with disturbance
        tau1 = P1 + D1 + disturbance[0]
        tau2 = P2 + D2 + disturbance[1]
        tau3 = P3 + D3 + disturbance[2]

        # Update joint velocities using numerical integration for unit moment of inertia
        qd1[i] = qd1[i - 1] + tau1 * dt
        qd2[i] = qd2[i - 1] + tau2 * dt
        qd3[i] = qd3[i - 1] + tau3 * dt

        # Update joint positions using numerical integration
        q1[i] = q1[i - 1] + qd1[i] * dt
        q2[i] = q2[i - 1] + qd2[i] * dt
        q3[i] = q3[i - 1] + qd3[i] * dt

        prev_error1 = error1
        prev_error2 = error2
        prev_error3 = error3


elif inp =='2':
    # Joint angles and velocities
    q2 = np.zeros_like(t)
    q1 = np.zeros_like(t)
    q3 = np.zeros_like(t)
    qd1 = np.zeros_like(t)
    qd2 = np.zeros_like(t)
    qd3 = np.zeros_like(t)

    prev_error1 = 0
    prev_error2 = 0
    prev_error3 = 0

    # PD control loop
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        error1 = q1_desired[i] - q1[i - 1]
        error2 = q2_desired[i] - q2[i - 1]
        error3 = q3_desired[i] - q3[i - 1]
    
        # Proportional and Derivative terms
        P1 = kp * error1
        P2 = kp * error2
        P3 = kp * error3

        D1 = kd * (error1 - prev_error1) / dt
        D2 = kd * (error2 - prev_error2) / dt
        D3 = kd * (error3 - prev_error3) / dt

        # Feedforward control terms
        F1 = kf * q1_desired[i]
        F2 = kf * q2_desired[i]
        F3 = kf * q3_desired[i]

        # PD control law with disturbance
        tau1 = P1 + D1 + F1 + disturbance[0]
        tau2 = P2 + D2 + F2 + disturbance[1]
        tau3 = P3 + D3 + F3 + disturbance[2]

        # Update joint velocities using numerical integration for unit moment of inertia
        qd1[i] = qd1[i - 1] + tau1 * dt
        qd2[i] = qd2[i - 1] + tau2 * dt
        qd3[i] = qd3[i - 1] + tau3 * dt

        # Update joint positions using numerical integration
        q1[i] = q1[i - 1] + qd1[i] * dt
        q2[i] = q2[i - 1] + qd2[i] * dt
        q3[i] = q3[i - 1] + qd3[i] * dt

        prev_error1 = error1
        prev_error2 = error2
        prev_error3 = error3   

elif inp =='3':
    qdd_1 = 0
    qdd_2 = 0
    qdd_3 = 0

    # Define symbolic variables
    q1, q2, q3, qd1, qd2, qd3, qdd1, qdd2, qdd3 = symbols('q1 q2 q3 qd1 qd2 qd3 qdd1 qdd2 qdd3')

    tau1 = 2*qd1*qd2*(0.25*(-0.3*sin(q2) - 0.15*sin(q2 + q3))*(0.15*cos(q2) + 0.075*cos(q2 + q3)) - 0.0028125*sin(q2)*cos(q2)) - 0.075*qd1*qd3*(0.15*cos(q2) + 0.075*cos(q2 + q3))*sin(q2 + q3) + qdd1*(0.01125*(cos(q2) + 0.5*cos(q2 + q3))**2 + 0.0028125*cos(q2)**2 + 0.03)
    tau2 = qd1**2*(-0.25*(-0.3*sin(q2) - 0.15*sin(q2 + q3))*(0.15*cos(q2) + 0.075*cos(q2 + q3)) + 0.0028125*sin(q2)*cos(q2)) + 0.0028125*qd2**2*sin(q2)*cos(q2) - 0.01125*qd2*qd3*sin(q3) - 0.005625*qd3**2*sin(q3) + qdd2*(0.0028125*sin(q2)**2 + 0.01125*cos(q3) + 0.0340625) + qdd3*(0.005625*cos(q3) + 0.0128125) + 1.105875*cos(q2)
    tau3 = 0.0375*qd1**2*(0.15*cos(q2) + 0.075*cos(q2 + q3))*sin(q2 + q3) + 0.005625*qd2**2*sin(q3) + qdd2*(0.005625*cos(q3) + 0.0128125) + 0.0128125*qdd3 + 0.368625*cos(q3)

    # Joint angles and velocities
    q_1 = np.zeros_like(t)
    q_2 = np.zeros_like(t)
    q_3 = np.zeros_like(t)
    qd_1 = np.zeros_like(t)
    qd_2 = np.zeros_like(t)
    qd_3 = np.zeros_like(t)

    prev_error1 = 0
    prev_error2 = 0
    prev_error3 = 0

    # PD control loop
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]

        # Substitute specific values
        values = {q1: q_1[i-1], q2: q_2[i-1], q3: q_3[i-1], qd1: qd_1[i-1], qd2: qd_2[i-1], qd3: qd_3[i-1], qdd1: qdd_1, qdd2: qdd_2, qdd3: qdd_3}

        tau_1 = tau1.subs(values)
        tau_2 = tau2.subs(values)
        tau_3 = tau3.subs(values)

        error1 = q1_desired[i] - q_1[i - 1]
        error2 = q2_desired[i] - q_2[i - 1]
        error3 = q3_desired[i] - q_3[i - 1]
    
        error1_dot = (error1 - prev_error1) / dt
        error2_dot = (error2 - prev_error2) / dt
        error3_dot = (error3 - prev_error3) / dt

        # Proportional and Derivative terms
        P1 = kp * error1
        P2 = kp * error2
        P3 = kp * error3

        D1 = kd * error1_dot
        D2 = kd * error2_dot
        D3 = kd * error3_dot

        # Feedforward control terms
        F1 = kf * q1_desired[i]
        F2 = kf * q2_desired[i]
        F3 = kf * q3_desired[i]

        # PD control law with disturbance
        qdd_1 = P1 + D1 + F1 + disturbance[0] + tau_1
        qdd_2 = P2 + D2 + F2 + disturbance[1] + tau_2
        qdd_3 = P3 + D3 + F3 + disturbance[2] + tau_3

        # Update joint velocities using numerical integration for unit moment of inertia
        qd_1[i] = qd_1[i - 1] + qdd_1 * dt
        qd_2[i] = qd_2[i - 1] + qdd_2 * dt
        qd_3[i] = qd_3[i - 1] + qdd_3 * dt

        # Update joint positions using numerical integration
        q_1[i] = q_1[i - 1] + qd_1[i] * dt
        q_2[i] = q_2[i - 1] + qd_2[i] * dt
        q_3[i] = q_3[i - 1] + qd_3[i] * dt

        prev_error1 = error1
        prev_error2 = error2
        prev_error3 = error3

else:
    print("Error in typing number code")


# Plotting the results
plt.figure(figsize=(12, 6))

plt.subplot(3, 1, 1)
plt.plot(t, q_1, label='Joint 1')
plt.plot(t, q1_desired, label='Desired Joint 1', linestyle='--')
plt.title('Joint Angles vs Time')
plt.xlabel('Time')
plt.ylabel('Joint Angle (q1)')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(t, q_2, label='Joint 2', color='orange')
plt.plot(t, q2_desired, label='Desired Joint 2', linestyle='--')
plt.xlabel('Time')
plt.ylabel('Joint Angle (q2)')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(t, q_3, label='Joint 3', color='green')
plt.plot(t, q3_desired, label='Desired Joint 3', linestyle='--')
plt.xlabel('Time')
plt.ylabel('Joint Angle (q3)')
plt.legend()

plt.tight_layout()
plt.show()