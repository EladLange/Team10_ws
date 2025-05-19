#!/usr/bin/env python3
"""
optimize_lap_time.py

Compute the minimum-lap-time racing line with CasADi.
"""

import numpy as np
import casadi as ca
import pandas as pd

# === Vehicle physics parameters ===
a_lat_max  = 8.0    # m/s², max lateral acceleration
a_long_max = 2.5    # m/s², max acceleration
a_long_min = -4.0   # m/s², max braking (negative)

# === File paths ===
CENTER_CSV = "/home/yonatan/Desktop/Team10_ws/src/race_line/baundary/Oval_path.csv"
INNER_CSV  = "/home/yonatan/Desktop/Team10_ws/src/race_line/baundary/Oval_path_baoundary0.csv"
OUTER_CSV  = "/home/yonatan/Desktop/Team10_ws/src/race_line/baundary/Oval_path_baoundary2.csv"
OUTPUT_CSV = "/home/yonatan/Desktop/Team10_ws/src/race_line/baundary/racing_line_min_time.csv"

# === 1. Load data ===
center = np.loadtxt(CENTER_CSV, delimiter=',', skiprows=1)
inner  = np.loadtxt(INNER_CSV,  delimiter=',', skiprows=1)
outer  = np.loadtxt(OUTER_CSV,  delimiter=',', skiprows=1)

N = len(center)
assert inner.shape[0] == N and outer.shape[0] == N, "Inner/outer must match center length"

# === 2. Pre-computations ===

# (a) Segment lengths ds
ds = np.linalg.norm(np.diff(center, axis=0), axis=1)
ds = np.hstack([ds, ds[-1]])  # close the loop

# (b) Tangents & normals
tangents = np.zeros_like(center)
tangents[1:-1] = center[2:] - center[:-2]
tangents[0]    = center[1] - center[0]
tangents[-1]   = center[-1] - center[-2]

normals = np.zeros_like(center)
for i in range(N):
    dx, dy = tangents[i]
    n = np.array([ dy, -dx ])
    L = np.linalg.norm(n)
    if L > 0:
        n /= L
    # ensure normal points outward toward outer boundary
    if np.dot(n, outer[i] - center[i]) < 0:
        n = -n
    normals[i] = n

# (c) Curvature of centerline
def compute_curvature(pts):
    x, y = pts[:,0], pts[:,1]
    dx, dy   = np.gradient(x), np.gradient(y)
    ddx, ddy = np.gradient(dx), np.gradient(dy)
    k = np.abs(dx*ddy - dy*ddx) / (dx*dx + dy*dy)**1.5
    return np.nan_to_num(k, nan=0.0, posinf=0.0, neginf=0.0)

curv = compute_curvature(center)

# (d) Offset bounds from center to boundaries
offset_min = np.zeros(N)
offset_max = np.zeros(N)
for i in range(N):
    off_in  = np.dot(inner[i]  - center[i], normals[i])
    off_out = np.dot(outer[i]  - center[i], normals[i])
    offset_min[i] = min(off_in, off_out)
    offset_max[i] = max(off_in, off_out)

# === 3. Set up optimization ===
opti = ca.Opti()
off = opti.variable(N)  # lateral offset
v   = opti.variable(N)  # speed

# lateral offset bounds
opti.subject_to(off >= offset_min)
opti.subject_to(off <= offset_max)

# positive speed
opti.subject_to(v >= 0.1)

# lateral-acceleration constraint only where curvature > 0
for i in range(N):
    if curv[i] > 1e-6:
        opti.subject_to(v[i]**2 * curv[i] <= a_lat_max)

# longitudinal accel/brake constraints
for i in range(N-1):
    opti.subject_to((v[i+1]-v[i]) * v[i] <=  a_long_max * ds[i])
    opti.subject_to((v[i+1]-v[i]) * v[i] >=  a_long_min * ds[i])

# close the loop
opti.subject_to(off[0] == off[-1])
opti.subject_to(v[0]   == v[-1])

# objective: minimize lap time = sum(ds/v)
time = 0
for i in range(N):
    time += ds[i] / v[i]
opti.minimize(time)

# initial guesses
opti.set_initial(off, 0)
v_guess = np.minimum(np.sqrt(a_lat_max / (curv + 1e-6)), 10.0)
opti.set_initial(v, v_guess)

# solver
opti.solver('ipopt', {"ipopt.print_level":0})
sol = opti.solve()

off_opt = sol.value(off)
v_opt   = sol.value(v)

# === 4. Construct racing line ===
racing = center + normals * off_opt[:,None]

# add constant Z = 0.2
zcol    = 0.2 * np.ones((N,1))
racing3 = np.hstack([racing, zcol])

# === 5. Save CSV ===
df = pd.DataFrame(racing3, columns=['x','y','z'])
df.to_csv(OUTPUT_CSV, index=False)
print("✅ Racing line saved to", OUTPUT_CSV)
print(f"⏱️ Estimated lap time: {sol.value(time):.3f} s")
