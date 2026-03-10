#!/usr/bin/env python3
"""
Digit Robot Animation from CSV Data
Reads joint positions and velocities from digit_cropped_data.csv and animates the robot
"""

import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import numpy as np
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.chdir("/home/saakee/Research/Saaketh_INEKF")  # data loader uses relative path
import digit_data_loader as dl
import time

# Load Digit MJCF model with floating base
urdf_path = "/home/saakee/Research/Saaketh_INEKF/digit_mujoco-main/src/digit_mujoco/models/flat/digit-v3-flat.xml"
# mesh_dir = "/home/saakee/Research/Saaketh_INEKF/digit_mujoco-main/src/digit_mujoco/models/stl"

print(f"Loading Digit model from: {urdf_path}")

# Load model with visual geometry
model, collision_model, visual_model = pin.buildModelsFromMJCF(urdf_path)
data = model.createData()

# print(f"\nModel loaded successfully!")
# print(f"Number of joints (nq): {model.nq}")
# print(f"Number of velocities (nv): {model.nv}")

# Data loaded by digit_data_loader
N = len(dl.timestamp)
print(f"Loaded {N} frames from digit_data_loader")

# Build name → numpy-array lookup (data loader uses underscores, model uses dashes)
dl_arrays = {attr: getattr(dl, attr) for attr in dir(dl)
             if not attr.startswith('_') and isinstance(getattr(dl, attr), np.ndarray)}

# Create mapping: model joint index → (pos_array, vel_array) from data loader
# Digit has a floating base: actual joints start at index 1, q[7+], v[6+]
joint_mapping = {}

for i in range(1, min(len(model.names)-1, 40)):
    joint_name = model.names[i+1]  # +1 because index 0 is universe
    dl_name = joint_name.replace('-', '_')      # e.g. "left_hip_roll"
    dl_vel_name = dl_name + '_vel'              # e.g. "left_hip_roll_vel"
    if dl_name in dl_arrays and dl_vel_name in dl_arrays:
        joint_mapping[i] = (dl_arrays[dl_name], dl_arrays[dl_vel_name])
    else:
        print(f"      ✗ No matching data loader array for: {joint_name}")

print(f"Successfully mapped {len(joint_mapping)} joints (excluding floating base)")

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
    viz.loadViewerModel(rootNodeName="digit_robot")
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
FRAME_ID = 9
frame_name = model.frames[FRAME_ID].name if FRAME_ID < len(model.frames) else "Unknown"
print (f"\nTracking velocity of frame {FRAME_ID}: {frame_name}")

# Animation parameters
fps = 30
dt = 1.0 / fps
speed_multiplier = 0.1  # Adjust to slow down or speed up the animation

print(f"\nStarting animation at {fps} fps (speed: {speed_multiplier}x)")
print(f"Total frames to animate: {N}")
print(f"Using {len(joint_mapping)} mapped joints")
print("Press Ctrl+C to stop\n")

# Foot position tracking state
contact_foot_pos_init = None  # World-frame position at contact onset

# Animation loop
try:
    for idx in range(N):
        # Extract joint positions and velocities for this frame
        q = np.zeros(model.nq)
        v = np.zeros(model.nv)
        
        # Populate floating base position (x, y, z) - first 3 elements of q
        q[0] = dl.pos_x[idx]
        q[1] = dl.pos_y[idx]
        q[2] = dl.pos_z[idx]
        
        # Populate floating base orientation (quaternion: w, x, y, z) - next 4 elements of q
        q[3] = dl.quat_w[idx]
        q[4] = dl.quat_i[idx]
        q[5] = dl.quat_j[idx]
        q[6] = dl.quat_k[idx]
        
        # Populate floating base linear velocity (vx, vy, vz) - first 3 elements of v
        v[0] = dl.vel_x[idx]
        v[1] = dl.vel_y[idx]
        v[2] = dl.vel_z[idx]
        
        # Populate floating base angular velocity (roll_rate, pitch_rate, yaw_rate) - next 3 elements of v
        v[3] = dl.r_rate[idx]
        v[4] = dl.p_rate[idx]
        v[5] = dl.y_rate[idx]
        
        # Fill in joint positions using the proper mapping
        # model_idx starts at 1 for first actual joint (after floating base)
        # In q: floating base takes indices 0-6 (3 pos + 4 quat)
        # So actual joints start at index 7, which is model_idx 1 -> q[7]
        for model_idx, (pos_arr, vel_arr) in joint_mapping.items():
            q[model_idx + 6] = pos_arr[idx]  # +6 because model_idx=1 → q[7]
            v[model_idx + 5] = vel_arr[idx]  # +5 because model_idx=1 → v[6]
        
        # For acceleration, we'll approximate it as zero (or could compute from velocity differences)
        a = np.zeros(model.nv)
        
        # Compute forward kinematics with velocities and accelerations
        pin.forwardKinematics(model, data, q, v, a)
        pin.updateFramePlacements(model, data)

        # Read left foot contact state from data loader
        left_contact = int(dl.lfoot_contact[idx])
        timestamp = dl.timestamp[idx]

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

# cd /home/saakee/Research/Saaketh_INEKF && /home/saakee/Research/Saaketh_INEKF/g1_env/bin/python Extra_Files/digit_csv_animation.py
