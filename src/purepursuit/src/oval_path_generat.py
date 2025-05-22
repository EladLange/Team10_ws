import numpy as np
import pandas as pd

# הגדרות
r = 50  # רדיוס
L = 200  # אורך הקטע הישר
N_line = 200  # נקודות לקווים ישרים
N_arc = 200   # נקודות לכל חצי עיגול

x_vals = []
y_vals = []
z_vals=[]

# קו תחתון ישר מ־(0, 0) ל־(L, 0)
x_vals += list(np.linspace(0, L, N_line))
y_vals += [0] * N_line
z_vals =0.0

# חצי עיגול ימני (עם מרכז ב־(L, r))
theta1 = np.linspace(-np.pi/2, np.pi/2, N_arc)
x_vals += list(L + r * np.cos(theta1))
y_vals += list(r + r * np.sin(theta1))
z_vals =0.0

# קו עליון ישר מ־(L, 2r) ל־(0, 2r)
x_vals += list(np.linspace(L, 0, N_line))
y_vals += [2*r] * N_line
z_vals =0.2

# חצי עיגול שמאלי (עם מרכז ב־(0, r))
theta2 = np.linspace(np.pi/2, 3*np.pi/2, N_arc)
x_vals += list(0 + r * np.cos(theta2))
y_vals += list(r + r * np.sin(theta2))
z_vals =0.2

# שמירה לקובץ CSV
df = pd.DataFrame({'x': x_vals, 'y': y_vals, 'z':z_vals})
df.to_csv("oval_path_wide.csv", index=False)
print("Saved oval_path.csv")
