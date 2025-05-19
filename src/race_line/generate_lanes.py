#!/usr/bin/env python3
"""
interactive_generate_lanes.py

Interactively ask for a base path CSV, number of lanes, and lane width,
then generate parallel lanes and save each to its own CSV.
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt  # Optional for debug plot

def load_base_path(csv_path):
    """
    Load base path from a CSV file with columns 'x' and 'y'.
    """
    df = pd.read_csv(csv_path)
    return df[['x', 'y']].values

def build_lanes(base_points, num_lanes, lane_width):
    """
    Generate parallel lanes using smoothed central-direction normals.
    """
    center_idx = num_lanes // 2
    lanes = [[] for _ in range(num_lanes)]

    # Compute central differences for tangent vector
    tangents = np.zeros_like(base_points)
    tangents[1:-1] = base_points[2:] - base_points[:-2]
    tangents[0] = base_points[1] - base_points[0]
    tangents[-1] = base_points[-1] - base_points[-2]

    # Compute normal vectors
    normals = np.zeros_like(base_points)
    for i in range(len(base_points)):
        dx, dy = tangents[i]
        normal = np.array([dy, -dx])
        norm_len = np.linalg.norm(normal)
        if norm_len > 0:
            normal /= norm_len
        normals[i] = normal

    # Build lanes by offsetting points
    for i, point in enumerate(base_points):
        for lane_idx in range(num_lanes):
            offset = (lane_idx - center_idx) * lane_width
            offset_point = point + offset * normals[i]
            lanes[lane_idx].append(offset_point)

    return [np.array(l) for l in lanes]

def save_lanes_to_csv(lanes, base_csv_path):
    """
    Save each lane to a separate CSV file named based on base_csv_path.
    """
    base_name = base_csv_path.rsplit('.', 1)[0]
    for idx, lane in enumerate(lanes):
        df_lane = pd.DataFrame(lane, columns=['x', 'y'])
        out_path = f"{base_name}_baoundary{idx}.csv"
        df_lane.to_csv(out_path, index=False)
        print(f"✅ Saved lane {idx} to {out_path}")

def plot_lanes(base, lanes):
    """
    Optional: Plot base path and all lanes.
    """
    plt.figure(figsize=(10, 6))
    plt.plot(base[:, 0], base[:, 1], 'k--', label="Base Path")
    for i, lane in enumerate(lanes):
        plt.plot(lane[:, 0], lane[:, 1], label=f"Lane {i}")
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.title("Generated Lanes")
    plt.show()

def main():
    # Interactive prompts
    input_csv = input("📂 Enter path to base CSV file (with x,y columns): ").strip()
    try:
        num_lanes = int(input("🛣️  Enter number of lanes to generate: ").strip())
    except ValueError:
        print("⚠️  Invalid number of lanes; defaulting to 3")
        num_lanes = 3
    try:
        lane_width = float(input("↔️  Enter lane width in meters: ").strip())
    except ValueError:
        print("⚠️  Invalid width; defaulting to 1.0")
        lane_width = 1.0

    # Generate and save lanes
    base_pts = load_base_path(input_csv)
    lanes = build_lanes(base_pts, num_lanes, lane_width)
    save_lanes_to_csv(lanes, input_csv)

    # Optional: plot lanes for verification
    should_plot = input("🖼️  Show plot of lanes? [y/N]: ").strip().lower()
    if should_plot == 'y':
        plot_lanes(base_pts, lanes)

if __name__ == "__main__":
    main()
