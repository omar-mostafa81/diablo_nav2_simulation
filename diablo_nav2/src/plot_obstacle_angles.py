import numpy as np
import matplotlib.pyplot as plt

# Load the angles from the file
angles = []
with open('angles.txt', 'r') as file:
    angles = [float(line.strip()) for line in file]

# Define the radius of the circle
radius = 1.0

# Compute x and y coordinates for each angle
x = [radius * np.cos(angle) for angle in angles]
y = [radius * np.sin(angle) for angle in angles]

# Define the middle angles of the free space (in degrees)
angle1_deg = 0
angle2_deg = 0.573

# Convert to radians
angle1_rad = np.deg2rad(angle1_deg)
angle2_rad = np.deg2rad(angle2_deg)

# Compute x and y coordinates for angle1 and angle2
angle1_x = radius * np.cos(angle1_rad)
angle1_y = radius * np.sin(angle1_rad)

angle2_x = radius * np.cos(angle2_rad)
angle2_y = radius * np.sin(angle2_rad)

# Define the middle angle of the free space (in degrees)
middle_angle1_deg = 180 + 0.573
middle_angle1_rad = np.deg2rad(middle_angle1_deg)
# Compute x and y coordinates for the middle angle
middle1_x = radius * np.cos(middle_angle1_rad)
middle1_y = radius * np.sin(middle_angle1_rad)

# # Define the middle angle of the free space (in degrees)
# middle_angle2_deg = 92.5
# middle_angle2_rad = np.deg2rad(middle_angle2_deg)
# # Compute x and y coordinates for the middle angle
# middle2_x = radius * np.cos(middle_angle2_rad)
# middle2_y = radius * np.sin(middle_angle2_rad)

# # Define the middle angle of the free space (in degrees)
# middle_angle3_deg = -102.5
# middle_angle3_rad = np.deg2rad(middle_angle3_deg)
# # Compute x and y coordinates for the middle angle
# middle3_x = radius * np.cos(middle_angle3_rad)
# middle3_y = radius * np.sin(middle_angle3_rad)

# # Define the middle angle of the free space (in degrees)
# middle_angle4_deg = 51.25
# middle_angle4_rad = np.deg2rad(middle_angle4_deg)
# # Compute x and y coordinates for the middle angle
# middle4_x = radius * np.cos(middle_angle4_rad)
# middle4_y = radius * np.sin(middle_angle4_rad)

# # Define the middle angle of the free space (in degrees)
# middle_angle5_deg = 133.75
# middle_angle5_rad = np.deg2rad(middle_angle5_deg)
# # Compute x and y coordinates for the middle angle
# middle5_x = radius * np.cos(middle_angle5_rad)
# middle5_y = radius * np.sin(middle_angle5_rad)

# # Define the middle angle of the free space (in degrees)
# middle_angle6_deg = -61.25
# middle_angle6_rad = np.deg2rad(middle_angle6_deg)
# # Compute x and y coordinates for the middle angle
# middle6_x = radius * np.cos(middle_angle6_rad)
# middle6_y = radius * np.sin(middle_angle6_rad)

# # Define the middle angle of the free space (in degrees)
# middle_angle7_deg = -143.75
# middle_angle7_rad = np.deg2rad(middle_angle7_deg)
# # Compute x and y coordinates for the middle angle
# middle7_x = radius * np.cos(middle_angle7_rad)
# middle7_y = radius * np.sin(middle_angle7_rad)

# Create the plot
plt.figure(figsize=(8, 8))
plt.scatter(x, y, c='blue', label='Obstacle Angles')

# Plot the middle angle as an arrow
plt.arrow(0, 0, middle1_x, middle1_y, head_width=0.05, head_length=0.1, fc='red', ec='red',  label='Middle Angle')
plt.text(middle1_x / 2 - 0.5, (middle1_y / 2) - 0.1, 'Middle Angle: −179.427°', fontsize=12, color='red', ha='center', va='top')

# plt.arrow(0, 0, middle2_x, middle2_y, head_width=0.05, head_length=0.1, fc='red', ec='red')
# plt.text(middle1_x / 2 + 0.4, (middle2_y / 2) + 0.7, f'Middle Angle 2: {middle_angle2_deg}°', fontsize=12, color='red', ha='center', va='top')

# plt.arrow(0, 0, middle3_x, middle3_y, head_width=0.05, head_length=0.1, fc='red', ec='red')
# plt.text(middle3_x / 2 - 0.1, (middle3_y / 2) - 0.7, f'Middle Angle 3: {middle_angle3_deg}°', fontsize=12, color='red', ha='center', va='top')

# plt.arrow(0, 0, middle4_x, middle4_y, head_width=0.05, head_length=0.1, fc='red', ec='red')
# plt.text(middle4_x / 2 + 0.8, (middle4_y / 2) + 0.6, f'Middle Angle 4: {middle_angle4_deg}°', fontsize=12, color='red', ha='center', va='top')

# plt.arrow(0, 0, middle5_x, middle5_y, head_width=0.05, head_length=0.1, fc='red', ec='red')
# plt.text(middle5_x / 2 - 0.5, (middle5_y / 2) + 0.6, f'Middle Angle 5: {middle_angle5_deg}°', fontsize=12, color='red', ha='center', va='top')

# plt.arrow(0, 0, middle6_x, middle6_y, head_width=0.05, head_length=0.1, fc='red', ec='red')
# plt.text(middle6_x / 2 + 0.8, (middle6_y / 2) - 0.6, f'Middle Angle 6: {middle_angle6_deg}°', fontsize=12, color='red', ha='center', va='top')

# plt.arrow(0, 0, middle7_x, middle7_y, head_width=0.05, head_length=0.1, fc='red', ec='red')
# plt.text(middle7_x / 2 - 0.5, (middle7_y / 2) - 0.5, f'Middle Angle 7: {middle_angle7_deg}°', fontsize=12, color='red', ha='center', va='top')

# Plot angle1 and angle2 as points
plt.scatter([angle1_x, angle2_x], [angle1_y, angle2_y], c='darkblue', s=100, label='Edge Angles')

# Annotate angle1 and angle2
plt.text(angle1_x, angle1_y - 0.15, f'angle1: {angle1_deg}°', fontsize=12, color='darkblue', ha='center', va='top')
plt.text(angle2_x, angle2_y + 0.15, f'angle2: {angle2_deg}°', fontsize=12, color='darkblue', ha='center', va='bottom')

# Set axis limits to show the full box
plt.xlim(-1.5, 1.5)
plt.ylim(-1.5, 1.5)

plt.title('Obstacle Angles on a Circle')
plt.xlabel('X')
plt.ylabel('Y')
plt.axhline(0, color='gray', lw=0.5)
plt.axvline(0, color='gray', lw=0.5)
plt.gca().set_aspect('equal', adjustable='box')
plt.legend()
plt.grid(True)

# Save the plot to a file
output_file = "angles_plot_with_labels.png"
plt.savefig(output_file)

plt.show()
