#!/usr/bin/env python

import math

def generate_octagon_path(num_points=500, radius=5.0, z_height=0.0, center_x=0.0, center_y=0.0):
    """
    Generate octagon path points with approximately equal spacing
    
    Args:
        num_points: Total number of points for the entire octagon
        radius: Distance from center to vertices (circumradius)
        z_height: Z-coordinate for all points
        center_x: X-coordinate of octagon center
        center_y: Y-coordinate of octagon center
    """
    path_points = []
    
    # Calculate angle increment for each point
    angle_increment = 2.0 * math.pi / num_points
    
    for i in range(num_points):
        # Calculate angle for current point
        angle = i * angle_increment
        
        # For an octagon, we want to place points along the edges
        # Divide the circle into 8 segments for 8 sides
        segment_angle = 2.0 * math.pi / 8  # π/4 radians = 45 degrees per segment
        
        # Determine which side of the octagon this point belongs to
        segment_idx = int(angle / segment_angle)
        local_angle = angle % segment_angle
        
        # Calculate the angle bisecting the current side
        side_center_angle = segment_idx * segment_angle + segment_angle / 2.0
        
        # Calculate the angle to the current vertex
        vertex_angle = segment_idx * segment_angle
        
        # Calculate the next vertex angle
        next_vertex_angle = (segment_idx + 1) % 8 * segment_angle
        
        # Calculate points along the edge between two adjacent vertices
        # Use linear interpolation along the edge
        t = local_angle / segment_angle  # Normalized position along the edge (0 to 1)
        
        # Calculate the two vertices of the current edge
        v1_x = center_x + radius * math.cos(vertex_angle)
        v1_y = center_y + radius * math.sin(vertex_angle)
        
        v2_x = center_x + radius * math.cos(next_vertex_angle)
        v2_y = center_y + radius * math.sin(next_vertex_angle)
        
        # Interpolate between the two vertices
        x = v1_x + t * (v2_x - v1_x)
        y = v1_y + t * (v2_y - v1_y)
        z = z_height
        
        path_points.append((x, y, z))
    
    return path_points

def generate_regular_octagon_path(num_points=500, radius=5.0, z_height=0.0, center_x=0.0, center_y=0.0):
    """
    Generate regular octagon path with more precise calculation
    """
    path_points = []
    
    # Define the 8 vertices of the octagon
    vertices = []
    for i in range(8):
        angle = i * (2 * math.pi / 8)  # 8 sides -> 8 angles
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        vertices.append((x, y))
    
    # Calculate how many points per edge
    points_per_edge = num_points // 8
    remaining_points = num_points % 8  # Extra points to distribute
    
    total_points = 0
    
    for edge_idx in range(8):
        # Get start and end vertices of current edge
        start_vertex = vertices[edge_idx]
        end_vertex = vertices[(edge_idx + 1) % 8]
        
        # Calculate number of points for this edge
        edge_points = points_per_edge
        if edge_idx < remaining_points:
            edge_points += 1  # Distribute extra points to first few edges
        
        # Generate points along this edge
        for i in range(edge_points):
            t = float(i) / (edge_points - 1) if edge_points > 1 else 0.0
            
            x = start_vertex[0] + t * (end_vertex[0] - start_vertex[0])
            y = start_vertex[1] + t * (end_vertex[1] - start_vertex[1])
            z = z_height
            
            path_points.append((x, y, z))
            total_points += 1
    
    # If we still need more points due to rounding issues, add them to the first edge
    while len(path_points) < num_points:
        # Add one more point to the first edge
        start_vertex = vertices[0]
        end_vertex = vertices[1]
        t = float(len(path_points) - sum([num_points//8 + (1 if j < num_points%8 else 0) for j in range(0)]) + 1) / (points_per_edge + (1 if 0 < num_points%8 else 0))
        if t <= 1.0:
            x = start_vertex[0] + t * (end_vertex[0] - start_vertex[0])
            y = start_vertex[1] + t * (end_vertex[1] - start_vertex[1])
            z = z_height
            path_points.append((x, y, z))
    
    return path_points[:num_points]  # Ensure exact number of points

def generate_simple_octagon_path(num_points=500, radius=5.0, z_height=0.0, center_x=0.0, center_y=0.0):
    """
    Simplest approach: distribute points evenly around octagon perimeter
    """
    path_points = []
    
    # Calculate the 8 vertices of the regular octagon
    vertices = []
    for i in range(8):
        angle = i * (2 * math.pi / 8) - math.pi / 2  # Start from bottom for better visualization
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        vertices.append((x, y))
    
    # Calculate total number of points per edge
    points_per_edge = num_points // 8
    remainder = num_points % 8  # Remaining points to distribute
    
    for edge_idx in range(8):
        # Determine how many points this edge will have
        edge_point_count = points_per_edge
        if edge_idx < remainder:
            edge_point_count += 1
        
        # Get start and end vertices of this edge
        start_vertex = vertices[edge_idx]
        end_vertex = vertices[(edge_idx + 1) % 8]
        
        # Generate points along this edge
        for i in range(edge_point_count):
            # Parameter t goes from 0 to 1 along the edge
            if edge_point_count == 1:
                t = 0.5  # If only one point, put it in the middle
            else:
                t = float(i) / (edge_point_count - 1)
            
            x = start_vertex[0] + t * (end_vertex[0] - start_vertex[0])
            y = start_vertex[1] + t * (end_vertex[1] - start_vertex[1])
            z = z_height
            
            path_points.append((x, y, z))
    
    return path_points

def save_path_to_file(path_points, filename='octagon_path.txt'):
    """
    Save path coordinates to a text file
    Each line contains x, y, z coordinates separated by spaces
    """
    with open(filename, 'w') as f:
        for x, y, z in path_points:
            f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")
    
    print(f"Octagon path saved to {filename} with {len(path_points)} points")

def main():
    # Generate the octagon path points
    path_points = generate_simple_octagon_path(
        num_points=500,    # Approximately 500 points
        radius=5.0,        # Distance from center to vertices
        z_height=0.0,      # Z height
        center_x=0.0,      # Center X coordinate
        center_y=0.0       # Center Y coordinate
    )
    
    # Save to file
    save_path_to_file(path_points, 'octagon_path.txt')
    
    # Print summary
    print(f"\nGenerated regular octagon path:")
    print(f"- Radius: 5.0m (distance from center to vertices)")
    print(f"- Total points: {len(path_points)}")
    print(f"- Center: ({0.0}, {0.0})")
    
    # Print first few points as a preview
    print("\nFirst 5 points:")
    for i, (x, y, z) in enumerate(path_points[:5]):
        print(f"Point {i+1}: x={x:.3f}, y={y:.3f}, z={z:.3f}")
    
    # Print some points from different edges as a preview
    n = len(path_points)
    print(f"\nSample points from different sections:")
    for idx in [n//8, n//4, n*3//8, n//2, n*5//8]:
        if idx < len(path_points):
            x, y, z = path_points[idx]
            print(f"Point {idx+1}: x={x:.3f}, y={y:.3f}, z={z:.3f}")

if __name__ == "__main__":
    main()