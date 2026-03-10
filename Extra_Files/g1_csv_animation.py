#!/usr/bin/env python3
"""
G1 Robot Animation from CSV Data
Reads joint positions and velocities from G1LoggingData.csv and animates the robot
"""

import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import numpy as np
import pandas as pd
import time

# Load G1 URDF model with floating base
urdf_path = "/home/saakee/Research/Saaketh_INEKF/src/utility/G1/g1_29dof_floatingBase.urdf"
mesh_dir = "unitree_ros/robots/g1_description"

print(f"Loading G1 model from: {urdf_path}")

# Load model with visual geometry
model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_path, mesh_dir)
data = model.createData()

# print(f"\nModel loaded successfully!")
# print(f"Number of joints (nq): {model.nq}")
# print(f"Number of velocities (nv): {model.nv}")

# Load CSV data
csv_path = "/home/saakee/Research/Saaketh_INEKF/G1LoggingData.csv"
# print(f"\nLoading CSV data from: {csv_path}")
df = pd.read_csv(csv_path)
# print(f"Loaded {len(df)} rows of data")

# Extract joint position column names (starting with "joint_pos_")
joint_pos_cols = [col for col in df.columns if col.startswith('joint_pos_')]
joint_vel_cols = [col for col in df.columns if col.startswith('joint_vel_')]

# print(f"\nFound {len(joint_pos_cols)} joint position columns")
# print(f"Found {len(joint_vel_cols)} joint velocity columns")

# Create mapping between model joints and CSV columns
# print(f"\nCreating joint mapping (skipping floating base)...")
# print(f"Model has {model.nq} DOF (including 7 for floating base)")
joint_mapping = {}  # Maps model joint index to CSV column name

# Check if first joint is floating base
first_joint_name = model.names[1] if len(model.names) > 1 else ""
has_floating_base = "floating" in first_joint_name.lower()

if has_floating_base:
    print(f"  [0] {first_joint_name} - Using pos_x/y/z and quat_w/i/j/k from CSV")

# Map the actual joints (skip floating base if present)
start_idx = 1 if has_floating_base else 0
for i in range(start_idx, min(len(model.names)-1, 30)):  # Limit to 30 joints max
    joint_name = model.names[i+1]  # +1 because index 0 is universe
    # print(f"  [{i}] {joint_name}")
    
    # Try to find matching CSV column
    csv_col_name = f"joint_pos_{joint_name}"
    if csv_col_name in joint_pos_cols:
        joint_mapping[i] = csv_col_name
        # print(f"      ✓ Mapped to CSV column: {csv_col_name}")
    else:
        print(f"      ✗ No matching CSV column found for: {csv_col_name}")

# print(f"\nSuccessfully mapped {len(joint_mapping)} joints (excluding floating base)")

# Initialize MeshCat visualizer
print("\nInitializing MeshCat visualizer...")
viz = MeshcatVisualizer(model, collision_model, visual_model)

try:
    viz.initViewer(open=True)
except ImportError as err:
    print("Error while initializing the viewer. It seems you should install python meshcat")
    print(err)
    import sys
    sys.exit(0)

try:
    viz.loadViewerModel(rootNodeName="g1_robot")
    print("Robot model loaded in MeshCat viewer!")
except Exception as e:
    print(f"Warning: {e}")
    print("Continuing anyway...")

# Add sphere marker at left foot (frame 44) - color indicates contact state
import meshcat.geometry as g
import meshcat.transformations as tf
COLOR_CONTACT    = 0x00ff00  # Green  = left foot IN contact
COLOR_NO_CONTACT = 0xff0000  # Red    = left foot NOT in contact
viz.viewer["left_foot_marker"].set_object(g.Sphere(0.075), g.MeshLambertMaterial(color=COLOR_NO_CONTACT))
prev_contact_state = -1  # Force first-frame update

print("\n" + "="*60)
print("MeshCat viewer is now running!")
print("Open your browser to view the animation")
print("URL: http://127.0.0.1:7000/static/")
print("="*60)

# Specify the frame index to track velocity (frame 44 - left foot)
FRAME_ID = 19
frame_name = model.frames[FRAME_ID].name if FRAME_ID < len(model.frames) else "Unknown"
print (f"\nTracking velocity of frame {FRAME_ID}: {frame_name}")

# Animation parameters
fps = 30
dt = 1.0 / fps
speed_multiplier = 0.1  # Adjust to slow down or speed up the animation

print(f"\nStarting animation at {fps} fps (speed: {speed_multiplier}x)")
print(f"Total frames to animate: {len(df)}")
print(f"Using {len(joint_mapping)} mapped joints")
print("Press Ctrl+C to stop\n")

# Foot position tracking state
contact_foot_pos_init = None  # World-frame position at contact onset

# Animation loop
try:
    for idx, row in df.iterrows():
        # Extract joint positions and velocities for this frame
        q = np.zeros(model.nq)
        v = np.zeros(model.nv)
        
        # Populate floating base position (x, y, z) - first 3 elements of q
        q[0] = row['pos_x']
        q[1] = row['pos_y']
        q[2] = row['pos_z']
        
        # Populate floating base orientation (quaternion: w, x, y, z) - next 4 elements of q
        q[3] = row['quat_i']
        q[4] = row['quat_j']  # x component
        q[5] = row['quat_k']  # y component
        q[6] = row['quat_w']  # z component
        
        # Populate floating base linear velocity (vx, vy, vz) - first 3 elements of v
        v[0] = row['vel_x']
        v[1] = row['vel_y']
        v[2] = row['vel_z']
        
        # Populate floating base angular velocity (wx, wy, wz) - next 3 elements of v
        v[3] = row['omega_x']
        v[4] = row['omega_y']
        v[5] = row['omega_z']
        
        # Fill in joint positions using the proper mapping
        # model_idx starts at 1 for first actual joint (after floating base)
        # In q: floating base takes indices 0-6 (3 pos + 4 quat)
        # So actual joints start at index 7, which is model_idx 1 -> q[7]
        for model_idx, csv_col in joint_mapping.items():
            q[model_idx + 6] = row[csv_col]  # +6 because model_idx=1 should map to q[7] (1+6=7)
            # Get corresponding velocity column
            vel_col = csv_col.replace('joint_pos_', 'joint_vel_')
            if vel_col in df.columns:
                v[model_idx + 5] = row[vel_col]  # +5 because model_idx=1 should map to v[6] (1+5=6)
        
        # For acceleration, we'll approximate it as zero (or could compute from velocity differences)
        a = np.zeros(model.nv)
        
        # Compute forward kinematics with velocities and accelerations
        pin.forwardKinematics(model, data, q, v, a)
        pin.updateFramePlacements(model, data)

        # Read left foot contact state from CSV
        left_contact = int(row['lfoot-contact'])
        timestamp = row['timestamp'] if 'timestamp' in row else idx * dt

        # Update contact indicator colour only when state changes (avoid redundant updates)
        if left_contact != prev_contact_state:
            contact_color = COLOR_CONTACT if left_contact else COLOR_NO_CONTACT
            viz.viewer["left_foot_marker"].set_object(
                g.Sphere(0.075), g.MeshLambertMaterial(color=contact_color)
            )
            # Detect transition: no contact -> in contact — initialise position tracker
            if left_contact == 1:
                contact_foot_pos_init = data.oMf[FRAME_ID].translation.copy()
                print(f"Time: {timestamp:.3f}s | *** LEFT FOOT CONTACT START — position reset to zero ***")
            # Detect transition: in contact -> off contact
            elif prev_contact_state == 1 and left_contact == 0:
                contact_foot_pos_init = None
                print(f"Time: {timestamp:.3f}s | *** LEFT FOOT CONTACT BROKEN ***")
            prev_contact_state = left_contact

        # Compute left foot velocity and relative position in world frame during contact
        left_foot_vel = pin.getFrameVelocity(model, data, FRAME_ID, pin.ReferenceFrame.WORLD)
        if left_contact and contact_foot_pos_init is not None:
            v_x = left_foot_vel.linear[0]
            current_pos = data.oMf[FRAME_ID].translation
            rel_pos = current_pos - contact_foot_pos_init
            print(f"Time: {timestamp:.3f}s | Left foot v_x = {v_x:.4f} m/s | "
                  f"rel_pos = [{rel_pos[0]:.4f}, {rel_pos[1]:.4f}, {rel_pos[2]:.4f}] m")

        # Update visualization
        viz.display(q)

        # Update left foot marker position at frame 44
        left_foot_pos = data.oMf[FRAME_ID].translation
        viz.viewer["left_foot_marker"].set_transform(
            tf.translation_matrix(left_foot_pos)
        )

        time.sleep(dt / speed_multiplier)
    
    print("\n" + "="*60)
    print("Animation complete!")
    print("Viewer will stay open. Press Ctrl+C to exit.")
    print("="*60)
    
    # Keep viewer open
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("\n\nAnimation stopped by user.")
    print("Closing viewer...")

print("\nDone!")

# cd /home/saakee/Research/Saaketh_INEKF && /home/saakee/Research/Saaketh_INEKF/g1_env/bin/python Extra_Files/g1_csv_animation.py
