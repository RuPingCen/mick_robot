#!/usr/bin/env python

import math

def generate_sine_path():
    """
    Generate sine path points similar to the make_sine_path function
    """
    # 路径参数（要和Gazebo里一致）
    length = 20.0    # 总长度 20m
    amplitude = 3.0  # 振幅 3m
    frequency = 1.0  # 频率（完整波动次数）
    N = 200          # 点数
    frame_id = "odom"

    path_points = []

    for i in range(N):
        # X方向线性增长
        x = (i / float(N)) * length
        
        # Y方向正弦曲线
        y = amplitude * math.sin(2.0 * math.pi * frequency * i / float(N))
        z = 0.0  # Z方向固定为0

        path_points.append((x, y, z))

    return path_points

def save_path_to_file(path_points, filename='path.txt'):
    """
    Save path coordinates to a text file
    Each line contains x, y, z coordinates separated by spaces
    """
    with open(filename, 'w') as f:
        for x, y, z in path_points:
            f.write(f"{x:.6f} {y:.6f} {z:.6f}\n")
    
    print(f"Path saved to {filename} with {len(path_points)} points")

def main():
    # Generate the sine path points
    path_points = generate_sine_path()
    
    # Save to file
    save_path_to_file(path_points, 'sine_path.txt')
    
    # Print first few points as a preview
    print("\nFirst 5 points:")
    for i, (x, y, z) in enumerate(path_points[:5]):
        print(f"Point {i+1}: x={x:.3f}, y={y:.3f}, z={z:.3f}")

if __name__ == "__main__":
    main()
