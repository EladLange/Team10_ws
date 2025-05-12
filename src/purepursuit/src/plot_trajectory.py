import pandas as pd           # For reading CSV files
import matplotlib.pyplot as plt  # For plotting graphs

# Load the original reference path and the Pure Pursuit generated trajectory
path = pd.read_csv("/home/yonatan/Motion-Planning-Team10/src/purepursuit/src/path.csv")          # Contains reference waypoints (x, y)
traj = pd.read_csv("/home/yonatan/Motion-Planning-Team10/src/purepursuit/src/trajectory.csv")    # Contains simulated path by the vehicle

# Create a new figure for plotting
plt.figure(figsize=(10, 5))

# Plot the original reference path (dashed line)
plt.plot(path["x"], path["y"], '--', label="Target Path")

# Plot the trajectory followed by the Pure Pursuit controller (solid line)
plt.plot(traj["x"], traj["y"], '-', label="Pure Pursuit Trajectory")

# Label axes and title
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("Pure Pursuit Simulation - Path Tracking")

# Add grid and legend for clarity
plt.grid(True)
plt.legend()
plt.axis("equal")  # Equal scaling for x and y
plt.show()         # Display the plot window

