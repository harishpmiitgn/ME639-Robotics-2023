import numpy as np

def calculate_joint_velocities():
    rows = int(input("Enter the number of rows in the Jacobian matrix: "))
    columns = int(input("Enter the number of columns in the Jacobian matrix: "))
    
    print("\nEnter the elements of the Jacobian matrix:")
    jacobian = np.array([[float(input(f"\tElement for row {i + 1}, column {j + 1}: ")) for j in range(columns)] for i in range(rows)])
    
    # User input for end-effector velocities
    print("\nEnter end-effector velocity for")
    end_effector_velocities = np.array([float(input(f"\tdirection {i + 1}: ")) for i in range(rows)])
    
    # Calculate joint velocities using the Jacobian matrix
    joint_velocities = np.linalg.pinv(jacobian) @ end_effector_velocities.reshape((-1, 1))
    
    return joint_velocities.flatten()

joint_velocities = calculate_joint_velocities()

print("\nJoint Velocities:")
print(joint_velocities)
