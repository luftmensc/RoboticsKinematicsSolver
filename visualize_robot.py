import sys
import matplotlib.pyplot as plt
import numpy as np

def plot_robot(joint_angles, end_effector_position, plot_title, ax=None):
    if ax is None:
        ax = plt.gca()
    
    link_lengths = [1.0, 1.0, 1.0]  # Example link lengths
    x = [0]
    y = [0]
    colors = ['r', 'g', 'b']  # Red, Green, Blue for the three links

    total_angle = 0
    for i, angle in enumerate(joint_angles):
        total_angle += angle
        x.append(x[i] + link_lengths[i] * np.cos(np.radians(total_angle)))
        y.append(y[i] + link_lengths[i] * np.sin(np.radians(total_angle)))
        ax.plot(x[i:i+2], y[i:i+2], '-o', color=colors[i], linewidth=3, markersize=8)

    ax.plot(end_effector_position[0], end_effector_position[1], 'rx', label='End Effector', markersize=10)
    ax.set_title(plot_title)

def setup_plot(ax=None):
    if ax is None:
        ax = plt.gca()
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_xlim(-3.2, 3.2)
    ax.set_ylim(-3.2, 3.2)
    ax.grid(True)
    ax.set_aspect('equal')

def display_plot():
    plt.legend()
    plt.show()

def inverse_kinematics(joint_angles, end_effector_position):
    # Create the figure once here
    plt.figure(figsize=(10, 5))

    # Plot first solution in the first subplot
    plt.subplot(1, 2, 1)  # (1 row, 2 columns, first subplot)
    setup_plot()  # No longer creates a new figure
    plot_robot(joint_angles[:3], end_effector_position, 'Inverse Kinematics - Solution 1')
    plt.legend()
    # Plot second solution in the second subplot
    plt.subplot(1, 2, 2)  # (1 row, 2 columns, second subplot)
    setup_plot()  # No longer creates a new figure
    plot_robot(joint_angles[3:], end_effector_position, 'Inverse Kinematics - Solution 2')
    plt.legend()
    # Call to display_plot() moved to the end of the inverse_kinematics function
    display_plot()

def forward_kinematics(joint_angles, end_effector_position):
    setup_plot()
    plot_robot(joint_angles, end_effector_position, 'Forward Kinematics')
    display_plot()

def inCircleTest(circle_center_x, circle_center_y, circle_radius, end_effector_x, end_effector_y, theta_1, theta_2, theta_3):
    # Calculate the distance between the circle center and the end effector
    distance = np.sqrt((circle_center_x - end_effector_x)**2 + (circle_center_y - end_effector_y)**2)

    # Check if the end effector is inside the circle
    inside = distance <= circle_radius
    status_message = "Inside Circle" if inside else "Outside Circle"

    # Link lengths
    link_lengths = [1.0, 1.0, 1.0]

    # Calculate joint positions based on thetas
    joint_positions = [[0, 0]]  # Base at origin
    for i, theta in enumerate([theta_1, theta_2, theta_3], start=1):
        prev_x, prev_y = joint_positions[-1]
        angle = np.radians(theta)
        x = prev_x + link_lengths[i-1] * np.cos(angle)
        y = prev_y + link_lengths[i-1] * np.sin(angle)
        joint_positions.append([x, y])

    # Create the figure and the axes
    fig, ax = plt.subplots()

    # Draw the circle
    circle = plt.Circle((circle_center_x, circle_center_y), circle_radius, color='blue', fill=False, label='Circle Boundary')
    ax.add_artist(circle)

    # Draw the robotic arm
    for i in range(len(joint_positions)-1):
        x_values = [joint_positions[i][0], joint_positions[i+1][0]]
        y_values = [joint_positions[i][1], joint_positions[i+1][1]]
        ax.plot(x_values, y_values, 'o-', label=f'Link {i+1}')

    # Set the limits of the plot
    ax.set_xlim(circle_center_x - circle_radius - 2, circle_center_x + circle_radius + 2)
    ax.set_ylim(circle_center_y - circle_radius - 2, circle_center_y + circle_radius + 2)

    # Add grid, legend, and equal aspect ratio
    ax.grid(True)
    ax.legend()
    ax.set_aspect('equal', 'box')

    # Display the status message on the plot
    plt.text(0.5, 0.95, status_message, ha='center', va='center', transform=ax.transAxes, fontsize=12, color='green' if inside else 'red')

    # Display the result
    plt.show()
    
def main():
    print("Incoming argument size: ", len(sys.argv))

    if len(sys.argv) == 6:
        # Get the end effector position and joint angles from command line arguments
        end_effector_x = float(sys.argv[1])
        end_effector_y = float(sys.argv[2])
        joint_angles = [float(arg) for arg in sys.argv[3:6]]
        # Call the forward kinematics function with the provided end effector position and joint angles
        forward_kinematics(joint_angles, [end_effector_x, end_effector_y])
    elif len(sys.argv) == 9:
        # Get the end effector position and joint angles from command line arguments
        end_effector_x = float(sys.argv[1])
        end_effector_y = float(sys.argv[2])
        joint_angles = [float(arg) for arg in sys.argv[3:9]]
        # Call the inverse kinematics function with the provided end effector position and joint angles
        inverse_kinematics(joint_angles, [end_effector_x, end_effector_y])
    elif len(sys.argv) == 10:
        circle_center_x = float(sys.argv[1])
        circle_center_y = float(sys.argv[2])
        circle_radius = float(sys.argv[3])
        end_effector_x = float(sys.argv[4])
        end_effector_y = float(sys.argv[5])
        theta_1 = float(sys.argv[6])
        theta_2 = float(sys.argv[7])
        theta_3 = float(sys.argv[8])
        inCircleTest(circle_center_x, circle_center_y, circle_radius, end_effector_x, end_effector_y, theta_1, theta_2, theta_3)


    else:
        print("Invalid number of arguments")
        sys.exit(1)


if __name__ == '__main__':
    main()
