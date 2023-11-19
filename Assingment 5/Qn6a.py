import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

def ur5_dynamics(t, y, link_lengths, masses, torque_values):
    q1, q2, q3, q1_dot, q2_dot, q3_dot = y
    
    l1, l2, l3 = link_lengths
    m1, m2, m3 = masses
    
    # Placeholder dynamics equations for UR5 
    D = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])  # Inertia matrix
    C = np.zeros((3, 3))  # Coriolis and centrifugal matrix
    G = np.array([0, 0, 0])  # Gravity vector
    tau = np.array(torque_values)  # Joint torques
    
    # Solve for joint accelerations
    q_dot = np.array([q1_dot, q2_dot, q3_dot])
    q_ddot = np.linalg.solve(D, tau - np.dot(C, q_dot) - G)
    
    # Return the derivatives
    return [q1_dot, q2_dot, q3_dot, q_ddot[0], q_ddot[1], q_ddot[2]]

# User input for link lengths
consider_lengths_as_one = input("Do you want to consider link lengths as 1? (Y/N): ").lower() == 'y'

if not consider_lengths_as_one:
    l1 = float(input("Enter length of link 1 (l1): "))
    l2 = float(input("Enter length of link 2 (l2): "))
    l3 = float(input("Enter length of link 3 (l3): "))
else:
    l1, l2, l3 = 1, 1, 1

# User input for masses
masses_equal_one = input("Do you want to consider masses equal to 0.1? (Y/N): ").lower() == 'y'

if masses_equal_one:
    m1, m2, m3 = 0.1, 0.1, 0.1
else:
    m1 = float(input("Enter mass of link 1 (m1): "))
    m2 = float(input("Enter mass of link 2 (m2): "))
    m3 = float(input("Enter mass of link 3 (m3): "))

# User input for torques
consider_torque_as_point_one = input("Do you want to consider torque as 0.1? (Y/N): ").lower() == 'y'

if consider_torque_as_point_one:
    tau_values = [0.1, 0.1, 0.1]
else:
    tau_values = [float(input(f"Enter torque value for joint {i + 1}: ")) for i in range(3)]

# Set initial conditions and time vector
initial_conditions = [0.1, 0.1, 0.1, 0.0, 0.0, 0.0]  # q1, q2, q3, q1_dot, q2_dot, q3_dot
time_span = (0, 10)
time_points = np.linspace(time_span[0], time_span[1], 1000)

# Solve the differential equations
solution = solve_ivp(ur5_dynamics, time_span, initial_conditions, args=((l1, l2, l3), (m1, m2, m3), tau_values), t_eval=time_points)

print("\n" + "-" * 150 + "\n")
print("\n")

# Plot each joint angle in a separate plot
plt.figure(figsize=(12, 8))

# Main title
plt.suptitle('UR5 Robot', fontsize=16)

# Plot q1
plt.subplot(3, 2, 1)
plt.plot(solution.t, np.degrees(solution.y[0, :]))
plt.title('Joint Angle q1 vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Joint Angle q1 (degrees)')

# Plot q2
plt.subplot(3, 2, 2)
plt.plot(solution.t, np.degrees(solution.y[1, :]))
plt.title('Joint Angle q2 vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Joint Angle q2 (degrees)')

# Plot q3
plt.subplot(3, 2, 3)
plt.plot(solution.t, np.degrees(solution.y[2, :]))
plt.title('Joint Angle q3 vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Joint Angle q3 (degrees)')

plt.tight_layout(rect=[0, 0.03, 1, 0.95])  
plt.show()
