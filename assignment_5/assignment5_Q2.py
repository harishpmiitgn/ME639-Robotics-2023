import numpy as np

def calculate_joint_velocities(jacobian_matrix, linear_velocities_end_effector, angular_velocities_end_effector):
    """
    Calculates joint velocities using the Jacobian matrix.

    Args:
    - jacobian_matrix: The Jacobian matrix of the robot.
    - linear_velocities_end_effector: End-effector linear velocities (m/s).
    - angular_velocities_end_effector: End-effector angular velocities (rad/s).

    Returns:
    - Tuple containing joint linear velocities and joint angular velocities.
    """
    try:
        # Combine linear and angular velocities into a single array
        end_effector_velocities = np.concatenate((linear_velocities_end_effector, angular_velocities_end_effector))
        
        # Calculate the pseudo-inverse of the Jacobian matrix
        jacobian_inv = np.linalg.pinv(jacobian_matrix)
        
        # Calculate joint velocities
        joint_velocities = np.dot(jacobian_inv, end_effector_velocities)
        
        # Separate joint velocities into linear and angular components
        return joint_velocities[:3], joint_velocities[3:]
    except np.linalg.LinAlgError:
        # Handle the case when the Jacobian is singular
        return None

# Get the number of links from user input
num_links_input = int(input("Please enter the number of links: \n"))

# Initialize the Jacobian matrix
jacobian_matrix_input = np.empty((6, num_links_input))

# Get user input for the Jacobian matrix
print("\nNow, kindly provide the elements of the Jacobian matrix row by row:")
for i in range(6):
    for j in range(num_links_input):
        jacobian_matrix_input[i][j] = float(input(f"Jacobian[{i}][{j}]: "))

# Get user input for end-effector velocities
linear_velocities_end_effector_input = np.array([float(k) for k in input("\nEnter the end-effector linear velocities (comma-separated) in meters per second: ").split(',')])
angular_velocities_end_effector_input = np.array([float(k) for k in input("Enter the end-effector angular velocities (comma-separated) in radians per second: ").split(',')])

# Calculate joint velocities
joint_linear_velocities_output, joint_angular_velocities_output = calculate_joint_velocities(jacobian_matrix_input, linear_velocities_end_effector_input, angular_velocities_end_effector_input)

# Display the results
if joint_linear_velocities_output is not None:
    print("\nHere are the calculated joint linear velocities:", joint_linear_velocities_output)
    print("And the joint angular velocities:", joint_angular_velocities_output)
else:
    print("\nApologies, but the Jacobian matrix is singular. Unable to compute joint velocities.")
