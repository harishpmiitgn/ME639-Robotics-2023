from math import *
import numpy as np

def manipulator_subroutine(num_links, dh_parameters, joint_types=None):
    # All joints are revolute if joint_types is not provided
    if joint_types is None:
        joint_types = ['R'] * num_links
    else:
        # Extend joint_types list if it's shorter than num_links
        joint_types.extend(['R'] * (num_links - len(joint_types)))

    DH = np.array(dh_parameters)

    H = []  # List of homogeneous transformation matrices
    for i in range(num_links):
        d, theta, a, alpha = DH[i]

        h = np.array([
            [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)],
            [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)],
            [0, sin(alpha), cos(alpha), d],
            [0, 0, 0, 1]
        ])

        H.append(h)

    H0 = []  # List of homogeneous transformation matrices H_0_i
    H0.append(H[0])
    h = H0[0]

    for i in range(1, num_links):
        h = np.matmul(h, H[i])
        H0.append(h)

    R = []  # List of Rotation matrices R_0_i
    O = []  # List of distance of the end point of ith joint from origin

    # O0 = zero
    zero = np.array([[0], [0], [0]])
    O.append(zero)

    Z = []
    k = np.array([[0], [0], [1]])
    Z.append(k)

    for i in range(num_links):
        r = H0[i][:3, :3]
        o = H0[i][:3, 3:4]
        R.append(r)
        O.append(o)
        z = np.matmul(r, k)
        Z.append(z)

    On = O[num_links]
    J = []  # List for columns of the Jacobian matrix for ith link

    for i in range(num_links):
        j = np.zeros((6, 1))

        # If revolute joint
        if joint_types[i] == 'R':
            J_v = np.cross(Z[i].flatten(), (On - O[i]).flatten())
            J_w = Z[i]
            j[:3, 0] = J_v
            j[3:, 0] = J_w.flatten()

        # If prismatic joint
        elif joint_types[i] == 'P':
            J_v = Z[i]
            J_w = zero
            j[:3, 0] = J_v.flatten()
            j[3:, 0] = J_w.flatten()

        J.append(j)

    Jacobian = J[0]
    for i in range(1, num_links):
        Jacobian = np.hstack((Jacobian, J[i]))

    On = np.round(On, decimals=2)
    Jacobian = np.round(Jacobian, decimals=2)

    return Jacobian, On

def end_effector_velocity(jacobian, joint_velocities):
    joint_velocities = np.array(joint_velocities).reshape((-1, 1))
    end_effector_vel = np.dot(jacobian, joint_velocities)
    return end_effector_vel

# input
num_links = int(input("Enter the number of links: "))
dh_parameters = []

for i in range(num_links):
    d = float(input(f"Enter d for link {i + 1}: "))
    theta = radians(float(input(f"Enter θ (degrees) for link {i + 1}: ")))
    a = float(input(f"Enter a for link {i + 1}: "))
    alpha = radians(float(input(f"Enter α (degrees) for link {i + 1}: ")))
    dh_parameters.append([d, theta, a, alpha])

joint_types_input = input("Enter joint types (R for Revolute, P for Prismatic), separated by spaces: ").split()

jacobian, end_effector_position = manipulator_subroutine(num_links, dh_parameters, joint_types_input)

print("\nManipulator Jacobian:")
print(jacobian)
print("\nEnd-effector Position:")
print(end_effector_position)

print("\n" + "-" * 50 + "\n")

# input for joint velocities
joint_velocities = []
for i in range(num_links):
    velocity = float(input(f"Enter joint velocity(q_dot) for joint {i + 1}: "))
    joint_velocities.append(velocity)

# Calculate end-effector velocity
end_effector_vel = end_effector_velocity(jacobian, joint_velocities)
print("\nEnd-effector Velocity:")
print(end_effector_vel)
