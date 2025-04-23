import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

# Load path and trajectory
path = pd.read_csv("/home/yonatan/Motion-Planning-Team10/src/purepursuit/src/path.csv")
traj = pd.read_csv("/home/yonatan/Motion-Planning-Team10/src/purepursuit/src/trajectory.csv")

# Prepare figure
fig, ax = plt.subplots(figsize=(10, 5))
ax.plot(path["x"], path["y"], '--', label="Target Path", color='gray')
line, = ax.plot([], [], '-', label="Vehicle Trajectory", color='blue')
point, = ax.plot([], [], 'ro', label="Vehicle Position")
arrow = ax.quiver([], [], [], [], color='red', scale=10)  # yaw arrow
info_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

# Set limits
ax.set_xlim(min(path["x"].min(), traj["x"].min()) - 5, max(path["x"].max(), traj["x"].max()) + 5)
ax.set_ylim(min(path["y"].min(), traj["y"].min()) - 5, max(path["y"].max(), traj["y"].max()) + 5)
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_title("Pure Pursuit - Live Animation with Heading and Info")
ax.grid(True)
ax.legend()
ax.set_aspect('equal')

def init():
    line.set_data([], [])
    point.set_data([], [])
    arrow.set_UVC([], [])
    info_text.set_text('')
    return line, point, arrow, info_text

def update(frame):
    x = traj["x"][frame]
    y = traj["y"][frame]
    yaw = traj["yaw"][frame]
    delta = traj["delta"][frame]

    # Draw trajectory and position
    line.set_data(traj["x"][:frame], traj["y"][:frame])
    point.set_data([x], [y])

    # Compute and draw heading arrow (yaw)
    dx = np.cos(yaw)
    dy = np.sin(yaw)
    arrow.set_offsets([[x, y]])
    arrow.set_UVC([dx], [dy])

    # Update text with velocity and delta
    info_text.set_text(f"\nVelocity: 10 m/s\nSteering angle (delta): {np.degrees(delta):.1f} [deg]")

    return line, point, arrow, info_text

ani = animation.FuncAnimation(
    fig, update, frames=len(traj),
    init_func=init, interval=50, blit=True, repeat=False)

plt.show()
