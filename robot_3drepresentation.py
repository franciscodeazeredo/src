import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sympy import symbols, cos, sin, pi

def plot_scorbot_vii(robot_pos, other_robot_pos, distance, joint_angles_robot1, joint_angles_robot2):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Define unique colors for each link
    link_colors = ['red', 'green', 'blue', 'purple', 'orange']

    # Define joint limits
    joint_limits = {
        'axis1': [-125, 125],
        'axis2': [-85, 85],
        'axis3': [-112.5, 112.5],
        'axis4': [-90, 90],
        'axis5': [-180, 180]
    }

    # Ensure joint angles stay within limits
    joint_angles_robot1 = [np.radians(np.clip(angle / 10, *joint_limits[f'axis{i + 1}'])) for i, angle in enumerate(joint_angles_robot1)]
    joint_angles_robot2 = [np.radians(np.clip(angle / 10, *joint_limits[f'axis{i + 1}'])) for i, angle in enumerate(joint_angles_robot2)]

    a1, d1, theta1, alpha1 = 0.05, 0.385, np.radians(joint_angles_robot1[0] / 10), 0
    a2, d2, theta2, alpha2 = 0.3, 0.385, np.radians(joint_angles_robot1[1] / 10), -np.pi/2
    a3, d3, theta3, alpha3 = 0.35, 0, np.radians(joint_angles_robot1[2] / 10), 0
    a4, d4, theta4, alpha4 = 0.251, 0, np.radians(joint_angles_robot1[3] / 10), 0
    a5, d5, theta5, alpha5 = 0, 0, np.radians(joint_angles_robot1[4] / 10), 0

    # Define transformation matrices for each joint
    def dh_matrix(alpha, a, d, theta):
        return np.array([
            [cos(theta), -sin(theta), 0, a],
            [sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -sin(alpha) * d],
            [sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), cos(alpha) * d],
            [0, 0, 0, 1]
        ])

    # Transformation matrices for each joint for the first robot
    T0_1_r1 = dh_matrix(alpha1, a1, d1, joint_angles_robot1[0])
    T1_2_r1 = dh_matrix(alpha2, a2, d2, joint_angles_robot1[1])
    T2_3_r1 = dh_matrix(alpha3, a3, d3, joint_angles_robot1[2])
    T3_4_r1 = dh_matrix(alpha4, a4, d4, joint_angles_robot1[3])
    T4_5_r1 = dh_matrix(alpha5, a5, d5, joint_angles_robot1[4])

    # Transformation matrices for each joint for the second robot
    T0_1_r2 = dh_matrix(alpha1, a1, d1, joint_angles_robot2[0])
    T1_2_r2 = dh_matrix(alpha2, a2, d2, joint_angles_robot2[1])
    T2_3_r2 = dh_matrix(alpha3, a3, d3, joint_angles_robot2[2])
    T3_4_r2 = dh_matrix(alpha4, a4, d4, joint_angles_robot2[3])
    T4_5_r2 = dh_matrix(alpha5, a5, d5, joint_angles_robot2[4])

    # Calculate final transformation matrix for each link for both robots
    T0_2_r1 = np.dot(T0_1_r1, T1_2_r1)
    T0_3_r1 = np.dot(T0_2_r1, T2_3_r1)
    T0_4_r1 = np.dot(T0_3_r1, T3_4_r1)
    T0_5_r1 = np.dot(T0_4_r1, T4_5_r1)

    T0_2_r2 = np.dot(T0_1_r2, T1_2_r2)
    T0_3_r2 = np.dot(T0_2_r2, T2_3_r2)
    T0_4_r2 = np.dot(T0_3_r2, T3_4_r2)
    T0_5_r2 = np.dot(T0_4_r2, T4_5_r2)

    # Extract link positions from transformation matrices for both robots
    base_pos_r1 = robot_pos
    base_pos_r2 = other_robot_pos

    # Applying transformations to link positions for the first robot
    link1_pos_r1 = np.dot(T0_1_r1, np.array([0, 0, 0, 1]))[:3] + base_pos_r1
    link2_pos_r1 = np.dot(T0_2_r1, np.array([0, 0, 0, 1]))[:3] + base_pos_r1
    link3_pos_r1 = np.dot(T0_3_r1, np.array([0, 0, 0, 1]))[:3] + base_pos_r1
    link4_pos_r1 = np.dot(T0_4_r1, np.array([0, 0, 0, 1]))[:3] + base_pos_r1
    link5_pos_r1 = np.dot(T0_5_r1, np.array([0, 0, 0, 1]))[:3] + base_pos_r1

    # Applying transformations to link positions for the second robot
    link1_pos_r2 = np.dot(T0_1_r2, np.array([0, 0, 0, 1]))[:3] + base_pos_r2
    link2_pos_r2 = np.dot(T0_2_r2, np.array([0, 0, 0, 1]))[:3] + base_pos_r2
    link3_pos_r2 = np.dot(T0_3_r2, np.array([0, 0, 0, 1]))[:3] + base_pos_r2
    link4_pos_r2 = np.dot(T0_4_r2, np.array([0, 0, 0, 1]))[:3] + base_pos_r2
    link5_pos_r2 = np.dot(T0_5_r2, np.array([0, 0, 0, 1]))[:3] + base_pos_r2


    # Plotting links for the first robot
    ax.plot([link1_pos_r1[0], link2_pos_r1[0]],
            [link1_pos_r1[1], link2_pos_r1[1]],
            [link1_pos_r1[2], link2_pos_r1[2]], color=link_colors[1], linewidth=2)

    ax.plot([link2_pos_r1[0], link3_pos_r1[0]],
            [link2_pos_r1[1], link3_pos_r1[1]],
            [link2_pos_r1[2], link3_pos_r1[2]], color=link_colors[2], linewidth=2)

    ax.plot([link3_pos_r1[0], link4_pos_r1[0]],
            [link3_pos_r1[1], link4_pos_r1[1]],
            [link3_pos_r1[2], link4_pos_r1[2]], color=link_colors[3], linewidth=2)

    ax.plot([link4_pos_r1[0], link5_pos_r1[0]],
            [link4_pos_r1[1], link5_pos_r1[1]],
            [link4_pos_r1[2], link5_pos_r1[2]], color=link_colors[4], linewidth=2)

    # Plotting joints for the first robot
    ax.scatter(*base_pos_r1, color='black', marker='x', label='Robot 1 Base')
    ax.scatter(*link1_pos_r1, color=link_colors[1], marker='o', label=f'Link 1 ({link1_pos_r1[0]:.2f}, {link1_pos_r1[1]:.2f}, {link1_pos_r1[2]:.2f})')
    ax.scatter(*link2_pos_r1, color=link_colors[2], marker='o', label=f'Link 2 ({link2_pos_r1[0]:.2f}, {link2_pos_r1[1]:.2f}, {link2_pos_r1[2]:.2f})')
    ax.scatter(*link3_pos_r1, color=link_colors[3], marker='o', label=f'Link 3 ({link3_pos_r1[0]:.2f}, {link3_pos_r1[1]:.2f}, {link3_pos_r1[2]:.2f})')
    ax.scatter(*link4_pos_r1, color=link_colors[4], marker='o', label=f'Link 4 ({link4_pos_r1[0]:.2f}, {link4_pos_r1[1]:.2f}, {link4_pos_r1[2]:.2f})')
    ax.scatter(*link5_pos_r1, color=link_colors[4], marker='o', label=f'Link 5 ({link5_pos_r1[0]:.2f}, {link5_pos_r1[1]:.2f}, {link5_pos_r1[2]:.2f})')

    # Plotting links for the second robot
    ax.plot([link1_pos_r2[0], link2_pos_r2[0]],
            [link1_pos_r2[1], link2_pos_r2[1]],
            [link1_pos_r2[2], link2_pos_r2[2]], color=link_colors[1], linewidth=2)

    ax.plot([link2_pos_r2[0], link3_pos_r2[0]],
            [link2_pos_r2[1], link3_pos_r2[1]],
            [link2_pos_r2[2], link3_pos_r2[2]], color=link_colors[2], linewidth=2)

    ax.plot([link3_pos_r2[0], link4_pos_r2[0]],
            [link3_pos_r2[1], link4_pos_r2[1]],
            [link3_pos_r2[2], link4_pos_r2[2]], color=link_colors[3], linewidth=2)

    ax.plot([link4_pos_r2[0], link5_pos_r2[0]],
            [link4_pos_r2[1], link5_pos_r2[1]],
            [link4_pos_r2[2], link5_pos_r2[2]], color=link_colors[4], linewidth=2)

    # Plotting joints for the second robot
    ax.scatter(*base_pos_r2, color='gray', marker='x', label='Robot 2 Base')
    ax.scatter(*link1_pos_r2, color=link_colors[1], marker='o', label=f'Link 1 ({link1_pos_r2[0]:.2f}, {link1_pos_r2[1]:.2f}, {link1_pos_r2[2]:.2f})')
    ax.scatter(*link2_pos_r2, color=link_colors[2], marker='o', label=f'Link 2 ({link2_pos_r2[0]:.2f}, {link2_pos_r2[1]:.2f}, {link2_pos_r2[2]:.2f})')
    ax.scatter(*link3_pos_r2, color=link_colors[3], marker='o', label=f'Link 3 ({link3_pos_r2[0]:.2f}, {link3_pos_r2[1]:.2f}, {link3_pos_r2[2]:.2f})')
    ax.scatter(*link4_pos_r2, color=link_colors[4], marker='o', label=f'Link 4 ({link4_pos_r2[0]:.2f}, {link4_pos_r2[1]:.2f}, {link4_pos_r2[2]:.2f})')
    ax.scatter(*link5_pos_r2, color=link_colors[4], marker='o', label=f'Link 5 ({link5_pos_r2[0]:.2f}, {link5_pos_r2[1]:.2f}, {link5_pos_r2[2]:.2f})')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Scorbot VII Representation')
    ax.legend()

    # Adjusting the view for better visibility
    ax.view_init(elev=0, azim=180)  # Set azim to 180 for facing each other

    # Set axis limits based on the total length
    ax.set_xlim([-0.821 - distance / 2, 0.821 + distance / 2])
    ax.set_ylim([-0.821, 0.821])
    ax.set_zlim([-0.25, 1.22])

    plt.show(block=True)

# Set the positions, distance between Scorbot VII robots, and joint angles for both robots
distance_between_robots = 4  # Adjust as needed
robot_position = np.array([distance_between_robots / 2, 0, 0])
other_robot_position = np.array([-distance_between_robots / 2, 0, 0])

# Joint angles for both robots
joint_angles_robot1 = [0, 0, 0, 0, 0]  # Adjust joint angles as needed for the first robot
joint_angles_robot2 = [0, 800, -900, 900, 0]  # Adjust joint angles as needed for the second robot

# Call the function with specified positions, distance, and joint angles for both robots
plot_scorbot_vii(robot_position, other_robot_position, distance_between_robots, joint_angles_robot1, joint_angles_robot2)







































