import matplotlib.pyplot as plt

# Function to read the intersection points from a file
def read_intersection_points(filename):
    points = []
    with open(filename, 'r') as file:
        lines = file.readlines()
        for line in lines:
            x, y = map(float, line.strip().split())
            points.append((x, y))
    return points

# Function to plot the polygon with annotations
def plot_polygon(points):
    if not points:
        print("No points to plot.")
        return

    # Close the polygon by appending the first point at the end
    points.append(points[0])

    # Unzip the list of points into x and y coordinates
    x, y = zip(*points)

    plt.figure(figsize=(12, 8))  # Increase the figure size
    plt.plot(x, y, marker='o')
    plt.fill(x, y, alpha=0.2)
    plt.title("Intersection Polygon")
    plt.xlabel("X coordinate (meters)")
    plt.ylabel("Y coordinate (meters)")
    plt.grid(True)
    
    # Set the limits to zoom in to the first 100 meters
    plt.xlim(-5, 5)
    plt.ylim(-5, 5)
    
    # Annotate each point with its coordinates
    for i in range(len(points) - 1):  # Exclude the duplicated last point
        plt.text(x[i], y[i], f'({x[i]:.1f}, {y[i]:.1f})', fontsize=9, ha='right')
    
    plt.show()

# Main function
if __name__ == "__main__":
    filename = "intersection_points.txt"
    points = read_intersection_points(filename)
    plot_polygon(points)
