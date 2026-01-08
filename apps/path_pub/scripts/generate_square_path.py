#!/usr/bin/env python

def generate_square_path(side_length=5.0, num_points_per_side=50, z_height=0.0, 
                         start_x=0.0, start_y=0.0, closed_path=True):
    """
    Generate square path points with customizable parameters
    
    Args:
        side_length: Length of each side of the square (meters)
        num_points_per_side: Number of points along each side
        z_height: Z-coordinate for all points
        start_x: Starting X-coordinate
        start_y: Starting Y-coordinate
        closed_path: Whether to close the path (connect end to start)
    """
    path_points = []
    
    # First side: Right (X increases)
    for i in range(num_points_per_side):
        x = start_x + (side_length * i / (num_points_per_side - 1))
        y = start_y
        z = z_height
        path_points.append((x, y, z))
    
    # Second side: Up (Y increases)
    for i in range(1, num_points_per_side):  # Skip first to avoid duplicate
        x = start_x + side_length
        y = start_y + (side_length * i / (num_points_per_side - 1))
        z = z_height
        path_points.append((x, y, z))
    
    # Third side: Left (X decreases)
    for i in range(1, num_points_per_side):  # Skip first to avoid duplicate
        x = start_x + side_length - (side_length * i / (num_points_per_side - 1))
        y = start_y + side_length
        z = z_height
        path_points.append((x, y, z))
    
    # Fourth side: Down (Y decreases)
    for i in range(1, num_points_per_side - 1):  # Skip last to avoid duplicate if closed
        x = start_x
        y = start_y + side_length - (side_length * i / (num_points_per_side - 1))
        z = z_height
        path_points.append((x, y, z))
    
    # Close the path if requested
    if closed_path and len(path_points) > 0:
        # Add the starting point again to close the loop
        path_points.append((start_x, start_y, z_height))
    
    return path_points

def save_path_to_file(path_points, filename='square_path.txt'):
    """
    Save path coordinates to a text file
    Each line contains x, y, z coordinates separated by spaces
    """
    with open(filename, 'w') as f:
        for x, y, z in path_points:
            f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")
    
    print(f"Square path saved to {filename} with {len(path_points)} points")

def main():
    # Generate the square path points with default parameters
    path_points = generate_square_path(
        side_length=5.0,           # 5米边长
        num_points_per_side=50,    # 每边50个点
        z_height=0.0,              # Z高度为0
        start_x=0.0,               # 起始X坐标
        start_y=0.0,               # 起始Y坐标
        closed_path=True           # 闭合路径
    )
    
    # Save to file
    save_path_to_file(path_points, 'square_path.txt')
    
    # Print summary
    print(f"\nGenerated square path:")
    print(f"- Side length: 5.0m")
    print(f"- Points per side: 50")
    print(f"- Total points: {len(path_points)}")
    print(f"- Closed path: True")
    
    # Print first few points as a preview
    print("\nFirst 5 points:")
    for i, (x, y, z) in enumerate(path_points[:5]):
        print(f"Point {i+1}: x={x:.3f}, y={y:.3f}, z={z:.3f}")
    
    # Print last few points as a preview
    print("\nLast 5 points:")
    for i, (x, y, z) in enumerate(path_points[-5:], len(path_points)-4):
        print(f"Point {i}: x={x:.3f}, y={y:.3f}, z={z:.3f}")

if __name__ == "__main__":
    main()