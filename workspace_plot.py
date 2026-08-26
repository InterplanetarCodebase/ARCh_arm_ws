import numpy as np
import matplotlib.pyplot as plt
from ikpy.chain import Chain

# 1. Load the robot arm directly from your URDF
urdf_path = "src/rover_arm_urdf/urdf/rover_arm_urdf.urdf"
arm_chain = Chain.from_urdf_file(urdf_path)

# 2. Define simulation parameters
num_samples = 25000  # Increased for better density in 2D views
x_vals, y_vals, z_vals, r_vals = [], [], [], []

# 3. Safely Extract Physical Limits
lower_bounds, upper_bounds = [], []
for link in arm_chain.links:
    if link.bounds is None:
        lower_bounds.append(0.0)
        upper_bounds.append(0.0)
    else:
        low, high = link.bounds
        if low is None or low == -np.inf: low = -np.pi
        if high is None or high == np.inf: high = np.pi
        if low > high: low, high = high, low
        lower_bounds.append(low)
        upper_bounds.append(high)

print("Calculating Forward Kinematics for 25,000 configurations...")

# 4. Monte Carlo Sampling
for _ in range(num_samples):
    random_angles = [np.random.uniform(low, high) for low, high in zip(lower_bounds, upper_bounds)]
    fk_matrix = arm_chain.forward_kinematics(random_angles)
    
    x = fk_matrix[0, 3]
    y = fk_matrix[1, 3]
    z = fk_matrix[2, 3]
    
    # Calculate radial distance from the base (for the side profile)
    r = np.sqrt(x**2 + y**2) 
    
    x_vals.append(x)
    y_vals.append(y)
    z_vals.append(z)
    r_vals.append(r)

# 5. Plotting the 3-Panel Dashboard
fig = plt.figure(figsize=(18, 6))

# --- Panel 1: 3D View ---
ax1 = fig.add_subplot(131, projection='3d')
ax1.scatter(x_vals, y_vals, z_vals, c=z_vals, cmap='viridis', s=1, alpha=0.15)
ax1.scatter([0], [0], [0], color='red', s=50, label="Base")
ax1.set_title("3D Workspace Cloud")
ax1.set_xlabel("X Axis (m)")
ax1.set_ylabel("Y Axis (m)")
ax1.set_zlabel("Z Axis (m)")

# --- Panel 2: Top-Down View (Swing Radius) ---
ax2 = fig.add_subplot(132)
ax2.scatter(x_vals, y_vals, c=z_vals, cmap='viridis', s=1, alpha=0.2)
ax2.scatter([0], [0], color='red', s=50, label="Base")
ax2.set_title("Top-Down View (X vs Y)")
ax2.set_xlabel("X Axis (m)")
ax2.set_ylabel("Y Axis (m)")
ax2.axis('equal') # Forces the grid to be perfectly square
ax2.grid(True, linestyle='--', alpha=0.6)

# --- Panel 3: Side Profile (Reach vs Height) ---
ax3 = fig.add_subplot(133)
scatter = ax3.scatter(r_vals, z_vals, c=z_vals, cmap='viridis', s=1, alpha=0.2)
ax3.scatter([0], [0], color='red', s=50, label="Base")
ax3.set_title("Side Profile (Max Reach vs Z Height)")
ax3.set_xlabel("Radial Distance from Base (m)")
ax3.set_ylabel("Z Height (m)")
ax3.grid(True, linestyle='--', alpha=0.6)

# Add a shared colorbar
cbar = fig.colorbar(scatter, ax=[ax1, ax2, ax3], shrink=0.8, pad=0.02)
cbar.set_label('Height (Z)')

plt.tight_layout()
plt.show()