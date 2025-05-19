#!/usr/bin/env python3
"""
interactive_generate_lanes.py

Interactively ask for a base path CSV, number of lanes, and lane width,
then generate parallel lanes and save each to its own CSV.
"""

import pandas as pd
import numpy as np

def load_base_path(csv_path):
    """
    Load base path from a CSV file with columns 'x' and 'y'.
    """
    df = pd.read_csv(csv_path)
    return df[['x', 'y']].values

def build_lanes(base_points, num_lanes, lane_width):
    """
    Given an array of base (x, y) points, generate num_lanes parallel lanes
    offset laterally by lane_width centered around the base path.
    Returns a list of numpy arrays for each lane.
    """
    center_idx = num_lanes // 2
    lanes = [ [] for _ in range(num_lanes) ]
    
    for i in range(len(base_points) - 1):
        p_curr = base_points[i]
        p_next = base_points[i + 1]
        dir_vec = p_next - p_curr
        normal = np.array([dir_vec[1], -dir_vec[0]])
        norm_len = np.hypot(normal[0], normal[1])
        if norm_len > 0:
            normal /= norm_len
        
        for lane_idx in range(num_lanes):
            offset = (lane_idx - center_idx) * lane_width
            lane_point = p_curr + normal * offset
            lanes[lane_idx].append(lane_point)
    
    return [np.array(l) for l in lanes]

def save_lanes_to_csv(lanes, base_csv_path):
    """
    Save each lane to a separate CSV file named based on base_csv_path.
    """
    base_name = base_csv_path.rsplit('.', 1)[0]
    for idx, lane in enumerate(lanes):
        df_lane = pd.DataFrame(lane, columns=['x', 'y'])
        out_path = f"{base_name}_lane{idx}.csv"
        df_lane.to_csv(out_path, index=False)
        print(f"Saved lane {idx} to {out_path}")

def main():
    # Interactive prompts
    input_csv = input("Enter path to base CSV file (with x,y columns): ").strip()
    try:
        num_lanes = int(input("Enter number of lanes to generate: ").strip())
    except ValueError:
        print("Invalid number of lanes; defaulting to 3")
        num_lanes = 3
    try:
        lane_width = float(input("Enter lane width in meters: ").strip())
    except ValueError:
        print("Invalid width; defaulting to 1.0")
        lane_width = 1.0

    # Generate and save lanes
    base_pts = load_base_path(input_csv)
    lanes = build_lanes(base_pts, num_lanes, lane_width)
    save_lanes_to_csv(lanes, input_csv)

if __name__ == "__main__":
    main()
