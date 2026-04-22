
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Example: 3 3D points
points = np.array([
    [-38.0, 10.0, 15.0],
    [-180.0, 10.0, 20.0],
    [-323.0, 10.0, 15.0]
])

sample_count = 30  # number of samples

# Parameter t: normalized between 0 and 1
t = np.linspace(0, 1, 3)
# Fit a 2nd degree polynomial for each axis
poly_x = np.polyfit(t, points[:, 0], 2)
poly_y = np.polyfit(t, points[:, 1], 2)
poly_z = np.polyfit(t, points[:, 2], 2)

# Sample points
t_sample = np.linspace(0, 1, sample_count)
x_sample = np.polyval(poly_x, t_sample)
y_sample = np.polyval(poly_y, t_sample)
z_sample = np.polyval(poly_z, t_sample)

def format_list(arr):
    return "[" + ", ".join(f"{v:.3f}".rstrip('0').rstrip('.') if '.' in f"{v:.3f}" else f"{v:.3f}" for v in arr) + "]"

print(f"    x: {format_list(x_sample)}")
print(f"    y: {format_list(y_sample)}")
print(f"    z: {format_list(z_sample)}")

# Plotting
fig = plt.figure(figsize=(16, 10))

# 3D plot
ax1 = fig.add_subplot(2, 2, 1, projection='3d')
ax1.plot(x_sample, y_sample, z_sample, label='Trajectory', color='b')
ax1.scatter(points[:, 0], points[:, 1], points[:, 2], color='r', label='Given Points')
ax1.set_title('3D Trajectory')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')
ax1.legend()

# XY projection
ax2 = fig.add_subplot(2, 2, 2)
ax2.plot(x_sample, y_sample, color='b')
ax2.scatter(points[:, 0], points[:, 1], color='r')
ax2.set_title('XY Projection')
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.axis('equal')

# XZ projection
ax3 = fig.add_subplot(2, 2, 3)
ax3.plot(x_sample, z_sample, color='b')
ax3.scatter(points[:, 0], points[:, 2], color='r')
ax3.set_title('XZ Projection')
ax3.set_xlabel('X')
ax3.set_ylabel('Z')
ax3.axis('equal')

# YZ projection
ax4 = fig.add_subplot(2, 2, 4)
ax4.plot(y_sample, z_sample, color='b')
ax4.scatter(points[:, 1], points[:, 2], color='r')
ax4.set_title('YZ Projection')
ax4.set_xlabel('Y')
ax4.set_ylabel('Z')
ax4.axis('equal')

plt.tight_layout()
plt.show()