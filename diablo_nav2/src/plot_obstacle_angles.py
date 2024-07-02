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

# Create the plot
plt.figure(figsize=(8, 8))
plt.scatter(x, y, c='blue', label='Obstacle Angles')
plt.title('Obstacle Angles on a Circle')
plt.xlabel('X')
plt.ylabel('Y')
plt.axhline(0, color='gray', lw=0.5)
plt.axvline(0, color='gray', lw=0.5)
plt.gca().set_aspect('equal', adjustable='box')
plt.legend()
plt.grid(True)

# Save the plot to a file
output_file = "angles_plot.png"
plt.savefig(output_file)

plt.show()
