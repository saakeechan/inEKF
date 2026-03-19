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

ENCODER_INDICES_MAP    = [7, 8, 9, 14, 15, 16, 17, 18, 23, 28, 29, 34, 35, 36, 41, 42, 43, 44, 45, 50, 55, 56]
ENCODERDOT_INDICES_MAP = [6, 7, 8, 12, 13, 14, 15, 16, 20, 24, 25, 30, 31, 32, 36, 37, 38, 39, 40, 44, 48, 49]

for joint_id in range(2, model.njoints):  # 0=universe, 1=floating_base
    joint_name = model.names[joint_id]
    dl_name = joint_name.replace('-', '_')      # e.g. "left_hip_roll"
    dl_vel_name = dl_name + '_vel'              # e.g. "left_hip_roll_vel"
    if dl_name in dl_arrays and dl_vel_name in dl_arrays:
        joint_mapping[joint_id] = (dl_arrays[dl_name], dl_arrays[dl_vel_name])
        # print(f"  ✓ Mapped joint '{joint_name}' to data loader arrays '{dl_name}' and '{dl_vel_name}'")
    else:
        print(f"      ✗ No matching data loader array for: {joint_name}")

for joint_id, (pos_arr, _) in sorted(joint_mapping.items()):
    jname = model.names[joint_id]
    iq = model.joints[joint_id].idx_q
    iv = model.joints[joint_id].idx_v
    eq = iq in ENCODER_INDICES_MAP
    ev = iv in ENCODERDOT_INDICES_MAP
    print(f"  joint_id={joint_id:2d}  {jname:35s}  q[{iq:2d}] {'✓' if eq else '✗'}  v[{iv:2d}] {'✓' if ev else '✗'}")

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
FRAME_ID = 31
frame_name = model.frames[FRAME_ID].name if FRAME_ID < len(model.frames) else "Unknown"
print (f"\nTracking velocity of frame {FRAME_ID}: {frame_name}")

# Animation parameters
fps = 30
dt = 1.0 / fps
speed_multiplier = 5  # Adjust to slow down or speed up the animation

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
        
        # # Set base pose from CSV data
        q[0:7] = dl.pos_x[idx], dl.pos_y[idx], dl.pos_z[idx] - 1.063*1.4, dl.quat_i[idx], dl.quat_j[idx], dl.quat_k[idx], dl.quat_w[idx]  # figure out z offset later
        
        # Convert world-frame velocities to body-frame for Pinocchio
        # Build rotation matrix from quaternion (quat format: x, y, z, w in q, but w, x, y, z in data)
        quat = pin.Quaternion(dl.quat_w[idx], dl.quat_i[idx], dl.quat_j[idx], dl.quat_k[idx])
        R = quat.toRotationMatrix()
        
        # World-frame velocities from data loader
        v_world = np.array([dl.vel_x[idx], dl.vel_y[idx], dl.vel_z[idx]])
        omega_world = np.array([dl.r_rate[idx], dl.p_rate[idx], dl.y_rate[idx]])
        
        # Transform to body frame: v_body = R^T * v_world
        v_body = R.T @ v_world
        omega_body = R.T @ omega_world
        
        # Populate floating base velocities in body frame
        v[0:3] = v_body
        v[3:6] = omega_body

        # q[2] = 1.0  # Set a fixed z height for better visualization (adjust as needed)
        # q[1] = 3.0  # Set a fixed y position for better visualization (adjust as needed)
        
        # Fill in joint positions using pinocchio's native idx_q / idx_v
        # (Digit has passive spring joints in between actuated joints, so indices are non-consecutive)
        for joint_id, (pos_arr, vel_arr) in joint_mapping.items():
            q[model.joints[joint_id].idx_q] = pos_arr[idx]
            v[model.joints[joint_id].idx_v] = vel_arr[idx]
        
        # Compute forward kinematics with velocities and accelerations
        pin.forwardKinematics(model, data, q, v)
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
        left_foot_vel = pin.getFrameVelocity(model, data, FRAME_ID, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        if left_contact and contact_foot_pos_init is not None:
            v_norm = np.linalg.norm(left_foot_vel.linear)
            current_pos = data.oMf[FRAME_ID].translation
            rel_pos = current_pos - contact_foot_pos_init
            print(f"Time: {timestamp:.3f}s | Left foot v_norm = {v_norm:.4f} m/s | "
                  f"world_pos = [{current_pos[0]:.4f}, {current_pos[1]:.4f}, {current_pos[2]:.4f}] m | ")
                #   f"rel_pos = [{rel_pos[0]:.4f}, {rel_pos[1]:.4f}, {rel_pos[2]:.4f}] m")

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
