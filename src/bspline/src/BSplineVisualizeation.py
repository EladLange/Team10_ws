import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Load B-spline evaluated points and tangent magnitudes + tangent vectors
data = pd.read_csv('bspline_curve.csv')
X, Y, Z = data['X'], data['Y'], data['Z']
dX, dY, dZ = data['dX'], data['dY'], data['dZ']
T = data['Tangent']

# Load control points (supporting 2D or 3D)
cp = pd.read_csv('controlPoints.csv')
if cp.shape[1] == 2:
    cp.columns = ['X', 'Y']
    cp['Z'] = 0.0
elif cp.shape[1] == 3:
    cp.columns = ['X', 'Y', 'Z']
else:
    raise ValueError("controlPoints.csv must have 2 or 3 columns")
CP_X, CP_Y, CP_Z = cp['X'], cp['Y'], cp['Z']

# Determine if 2D or 3D visualization
is3D = not np.allclose(Z, 0)

# Plot B-spline curve and tangent vectors
fig = plt.figure(figsize=(12, 6))
ax1 = fig.add_subplot(211, projection='3d' if is3D else None)

# Plot curve and control points
if is3D:
    ax1.plot3D(X, Y, Z, 'b-', label='B-spline Curve')
    ax1.plot3D(CP_X, CP_Y, CP_Z, 'ro--', label='Control Points')
else:
    ax1.plot(X, Y, 'b-', label='B-spline Curve')
    ax1.plot(CP_X, CP_Y, 'ro--', label='Control Points')

# Tangent arrows (every N points)
N = 50
for i in range(0, len(X), N):
    if is3D:
        ax1.quiver(X[i], Y[i], Z[i], dX[i], dY[i], dZ[i], color='g', linewidth=1)
    else:
        ax1.quiver(X[i], Y[i], dX[i], dY[i], angles='xy', scale_units='xy', scale=1, color='g', width=0.002)

ax1.set_title('Clamped Cubic B-spline Curve')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
if is3D:
    ax1.set_zlabel('Z')
h_curve, = ax1.plot([], [], 'b-', label='B-spline Curve')
h_ctrl, = ax1.plot([], [], 'ro--', label='Control Points')
h_tangent = ax1.quiver([], [], [], [], [], [], color='g') if is3D else ax1.quiver([], [], [], [], color='g')
ax1.legend([h_curve, h_ctrl, h_tangent], ['B-spline Curve', 'Control Points', 'Tangent Vectors'])
ax1.grid(True)

# Plot tangent magnitudes
ax2 = fig.add_subplot(212)
ax2.plot(T, 'm-', linewidth=2)
ax2.set_title('Tangent Vector Magnitude Along Curve')
ax2.set_xlabel('Point Index')
ax2.set_ylabel('Tangent')
ax2.grid(True)

plt.tight_layout()
plt.show()
