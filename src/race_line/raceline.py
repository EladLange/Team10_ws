"""
Module for computing the optimal racing line given track boundaries.

Configure the paths to your boundary CSVs and output directly in the code.

Each boundary CSV must have a header and two columns: x,y
"""
import numpy as np
from scipy.interpolate import splprep, splev

# Configuration: set your file paths here
INNER_CSV = "/home/yonatan/Desktop/Team10_ws/src/race_line/Oval_path_INNER.csv"    # Replace with your inner boundary CSV path
OUTER_CSV = "/home/yonatan/Desktop/Team10_ws/src/race_line/Oval_path_OUTER.csv"    # Replace with your outer boundary CSV path
OUTPUT_CSV = "/home/yonatan/Desktop/Team10_ws/src/race_line/racing_line_new.csv"  # Replace with desired output path


def load_boundary_csv(path):
    """
    Load a boundary CSV file with header "x,y" into an Nx2 numpy array.
    """
    try:
        data = np.loadtxt(path, delimiter=',', skiprows=1)
    except Exception as e:
        raise RuntimeError(f"Failed to load boundary CSV '{path}': {e}")
    if data.ndim != 2 or data.shape[1] != 2:
        raise ValueError(f"Boundary CSV '{path}' must have two columns x,y")
    return data


def compute_midline(inner, outer, n_points=500):
    """
    Compute a smooth midline by averaging inner and outer boundaries
    and fitting a spline through the midpoint sequence.
    """
    mid_raw = (inner + outer) / 2.0
    tck, _ = splprep([mid_raw[:,0], mid_raw[:,1]], s=0)
    u_fine = np.linspace(0, 1, n_points)
    x_smooth, y_smooth = splev(u_fine, tck)
    return np.vstack([x_smooth, y_smooth]).T


def compute_curvature(line):
    """
    Estimate curvature κ at each point of the line.
    κ = |x' y'' - y' x''| / (x'^2 + y'^2)^(3/2)
    """
    x, y = line[:,0], line[:,1]
    dx, dy = np.gradient(x), np.gradient(y)
    ddx, ddy = np.gradient(dx), np.gradient(dy)
    curvature = np.abs(dx * ddy - dy * ddx) / np.power(dx*dx + dy*dy, 1.5)
    return curvature


def optimize_racing_line(midline, inner, outer):
    """
    Compute a racing line by offsetting the midline:
    - Start hugging the outer boundary
    - Transition to hugging the inner boundary at the apex (max curvature)
    - Return to outer boundary after the apex
    """
    N = midline.shape[0]
    # Parameter values along the spline
    u = np.linspace(0, 1, N)
    # Resample inner and outer to N points
    tck_i, _ = splprep([inner[:,0], inner[:,1]], s=0)
    xi, yi = splev(u, tck_i)
    tck_o, _ = splprep([outer[:,0], outer[:,1]], s=0)
    xo, yo = splev(u, tck_o)
    boundary_inner = np.vstack([xi, yi]).T
    boundary_outer = np.vstack([xo, yo]).T
    # Direction vectors from inner to outer
    widths = np.linalg.norm(boundary_outer - boundary_inner, axis=1)
    dirs = (boundary_outer - boundary_inner) / widths[:, None]
    # Find apex index via curvature
    curvature = compute_curvature(midline)
    apex_idx = np.argmax(curvature)
    # Compute offset factor f: +1 at start->-1 at apex->+1 at end
    f = np.zeros(N)
    for i in range(N):
        if i <= apex_idx:
            f[i] = 1 - 2 * (i / apex_idx)
        else:
            f[i] = -1 + 2 * ((i - apex_idx) / (N - 1 - apex_idx))
    # Compute offsets and apply
    offsets = dirs * (widths / 2)[:, None] * f[:, None]
    raw_line = midline + offsets
    # Smooth the racing line
    tck, _ = splprep([raw_line[:,0], raw_line[:,1]], s=0)
    x_opt, y_opt = splev(u, tck)
    return np.vstack([x_opt, y_opt]).T


def save_line(line, path):
    """
    Save N×2 array of x,y points to a CSV file.
    """
    header = 'x,y'
    np.savetxt(path, line, delimiter=',', header=header, comments='')


def main():
    # Load boundaries
    inner = load_boundary_csv(INNER_CSV)
    outer = load_boundary_csv(OUTER_CSV)
    # Compute midline
    midline = compute_midline(inner, outer, n_points=500)
    # Optimize racing line
    racing_line = optimize_racing_line(midline, inner, outer)
    # Save result
    save_line(racing_line, OUTPUT_CSV)
    print(f"Racing line saved to {OUTPUT_CSV}")


if __name__ == '__main__':
    main()
