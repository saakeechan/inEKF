#!/usr/bin/env python3
"""
GRF Analysis using Pinocchio
Compute Ground Reaction Forces from joint torques using Jacobian method
"""

import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import os
import time
import sys
import matplotlib.pyplot as plt

# Add Extra_Files to path to import digit_data_loader
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

# Import digit data loader
import digit_data_loader as ddl

# =====================================================================
# STEP 1: Initialize Pinocchio - Load URDF and build robot model
# =====================================================================
model_path = "digit_mujoco-main/src/digit_mujoco/models/flat/digit-v3-flat.xml"

print("Loading robot model from:", model_path)
if not os.path.exists(model_path):
    print(f"ERROR: Model file not found at {model_path}")
    exit(1)

# Build model from URDF (Pinocchio supports .urdf, .xml (MuJoCo))
# Note: For MuJoCo XML files, you may need to convert to URDF first
# or use mujoco_parser if available
try:
    model, collision_model, visual_model = pin.buildModelsFromMJCF(model_path)
    data = model.createData()
    print(f"Model loaded successfully: {model.nq} DOF, {model.nv} velocities")
except Exception as e:
    print(f"Error loading model: {e}")
    print("If MuJoCo XML loading fails, you may need to convert to URDF first")
    exit(1)

# Define foot frame IDs directly
left_foot_id = 31  # Left foot frame
right_foot_id = 71  # Right foot frame

# =====================================================================
# Initialize Meshcat Visualizer
# =====================================================================
print("\nInitializing Meshcat visualizer...")
viz = MeshcatVisualizer(model, collision_model, visual_model)
viz.initViewer(open=True)
viz.loadViewerModel()
print("Meshcat visualizer ready! Open the URL in your browser.")

# =====================================================================
# STEP 2: Load CSV data using digit_data_loader
# =====================================================================
print(f"\nLoaded {len(ddl.timestamp)} time steps from digit_data_loader")
print(f"\nModel has {model.nq} position DOFs and {model.nv} velocity DOFs")

# =====================================================================
# STEP 3: Process each time step
# =====================================================================
print(f"\nProcessing {len(ddl.timestamp)} time steps...")

# Store results for later analysis
configurations = []  # Store all configurations for replay

# Define index mappings for encoder positions and velocities
encoder_indices_map = [7, 8, 9, 14, 15, 16, 17, 18, 23, 28, 29, 34, 35, 36, 41, 42, 43, 44, 45, 50, 55, 56]
encoderdot_indices_map = [6, 7, 8, 10, 11, 12, 13, 14, 16, 18, 19, 24, 25, 26, 28, 29, 30, 31, 32, 34, 36, 37]

# Create encoder arrays (22 encoders total) - ordered as in CSV
encoders_list = [
    ddl.left_hip_roll, ddl.left_hip_yaw, ddl.left_hip_pitch, ddl.left_knee,
    ddl.left_shin, ddl.left_tarsus, ddl.left_heel_spring,
    ddl.left_toe_A, ddl.left_toe_B, ddl.left_toe_pitch, ddl.left_toe_roll, 

    ddl.right_hip_roll, ddl.right_hip_yaw, ddl.right_hip_pitch, ddl.right_knee,
    ddl.right_shin, ddl.right_tarsus, ddl.right_heel_spring,
    ddl.right_toe_A, ddl.right_toe_B, ddl.right_toe_pitch, ddl.right_toe_roll, 
]

# Create encoderdot arrays (22 encoder velocities total) - ordered as in CSV
encoderdots_list = [
    ddl.left_hip_roll_vel, ddl.left_hip_yaw_vel, ddl.left_hip_pitch_vel, ddl.left_knee_vel,
    ddl.left_shin_vel, ddl.left_tarsus_vel, ddl.left_heel_spring_vel,
    ddl.left_toe_A_vel, ddl.left_toe_B_vel, ddl.left_toe_pitch_vel, ddl.left_toe_roll_vel,

    ddl.right_hip_roll_vel, ddl.right_hip_yaw_vel, ddl.right_hip_pitch_vel, ddl.right_knee_vel,
    ddl.right_shin_vel, ddl.right_tarsus_vel, ddl.right_heel_spring_vel,
    ddl.right_toe_A_vel, ddl.right_toe_B_vel, ddl.right_toe_pitch_vel, ddl.right_toe_roll_vel
]
normVectorLeft = []  # To store norms of GRF for analysis
normTorqueVecLeft = []  # To store norms of joint torques for analysis
normVectorRight = []  # To store norms of GRF for analysis
normTorqueVecRight = []  # To store norms of joint torques for analysis

for i in range(len(ddl.timestamp)):
    # Initialize full configuration vectors (all zeros)
    q = np.zeros(model.nq)
    q_dot = np.zeros(model.nv)
    
    # Extract encoder values and assign to specified indices in q
    for j, encoder_data in enumerate(encoders_list):
        q[encoder_indices_map[j]] = encoder_data[i]
    
    # Extract encoderdot values and assign to specified indices in q_dot
    for j, encoderdot_data in enumerate(encoderdots_list):
        q_dot[encoderdot_indices_map[j]] = encoderdot_data[i]
    
    # Run forward kinematics
    pin.forwardKinematics(model, data, q, q_dot)
    pin.updateFramePlacements(model, data)

    # Compute joint Jacobians (prepares data structure for efficient frame Jacobian queries)
    pin.computeJointJacobians(model, data, q)
    
    # Get frame Jacobians in WORLD frame (returns 6xN matrix)
    J_left_full = pin.getFrameJacobian(model, data, left_foot_id, pin.WORLD)
    left_foot_jacobian = J_left_full[:3, :]  # Extract linear velocity part (first 3 rows)
    
    J_right_full = pin.getFrameJacobian(model, data, right_foot_id, pin.WORLD)
    right_foot_jacobian = J_right_full[:3, :]  # Extract linear velocity part (first 3 rows)


    
    # Create a zero jacobian with same shape, then copy only specified columns
    active_left_foot_jacobian = np.zeros_like(left_foot_jacobian)  # 3x54
    active_left_foot_jacobian[:, encoderdot_indices_map] = left_foot_jacobian[:, encoderdot_indices_map]

    active_right_foot_jacobian = np.zeros_like(right_foot_jacobian)  # 3x54
    active_right_foot_jacobian[:, encoderdot_indices_map] = right_foot_jacobian[:, encoderdot_indices_map]
    
    # print(f"Time step {i}: Active left foot Jacobian shape: {active_left_foot_jacobian.shape}")
    # print(f"Time step {i}: Active right foot Jacobian (rows 0-2):\n{right_foot_jacobian}")
    # break  # Just check the first time step for now

    # Create separate torque lists for left and right legs
    # Passive joints (shin, tarsus, heel_spring, toe_pitch, toe_roll) have no torque data - set to zero
    left_torques_list = [
        ddl.left_hip_roll_tau, ddl.left_hip_yaw_tau, ddl.left_hip_pitch_tau, ddl.left_knee_tau,
        ddl.left_shin_vel * 0, ddl.left_tarsus_vel * 0, ddl.left_heel_spring_vel * 0, 
        ddl.left_toe_A_tau, ddl.left_toe_B_tau, ddl.left_toe_pitch_vel * 0, ddl.left_toe_roll_vel * 0
    ]
    
    right_torques_list = [
        ddl.right_hip_roll_tau, ddl.right_hip_yaw_tau, ddl.right_hip_pitch_tau, ddl.right_knee_tau,
        ddl.right_shin_vel * 0, ddl.right_tarsus_vel * 0, ddl.right_heel_spring_vel * 0,
        ddl.right_toe_A_tau, ddl.right_toe_B_tau, ddl.right_toe_pitch_vel * 0, ddl.right_toe_roll_vel * 0
    ]
    
    # Create separate tau vectors for left and right legs
    tau_left = np.zeros(model.nv)  # 54x1 zero vector
    tau_right = np.zeros(model.nv)  # 54x1 zero vector
    
    # First 11 indices in encoderdot_indices_map are for left leg
    for j in range(11):
        tau_left[encoderdot_indices_map[j]] = left_torques_list[j][i]
    
    # Last 11 indices in encoderdot_indices_map are for right leg
    for j in range(11):
        tau_right[encoderdot_indices_map[j + 11]] = right_torques_list[j][i]
    
    # Compute GRF using Jacobian transpose method: tau = J^T * F  =>  F = (J^T)^+ * tau
    J_T_Left = active_left_foot_jacobian.T  # 54x3
    F_left = np.linalg.pinv(J_T_Left) @ tau_left  # (3x54) @ (54x1) = 3x1

    J_T_Right = active_right_foot_jacobian.T  # 54x3
    F_right = np.linalg.pinv(J_T_Right) @ tau_right  # (3x54) @ (54x1) = 3x1

    normVectorLeft.append(np.linalg.norm(F_left))  # Just to check if we get a reasonable value
    normVectorRight.append(np.linalg.norm(F_right))  # Just to check if we get a reasonable value

    normTorqueVecLeft.append(np.linalg.norm(tau_left))  # Just to check if we get a reasonable value
    normTorqueVecRight.append(np.linalg.norm(tau_right))  # Just to check if we get a reasonable value

    # Store configuration for replay
    configurations.append(q.copy())



print(f"\nCompleted processing {len(ddl.timestamp)} time steps")
plt.figure()
plt.plot(normTorqueVecLeft, label='Left Foot')
plt.plot(normTorqueVecRight, label='Right Foot')
plt.xlabel('Time step')
plt.ylabel('Norm of Joint Torques')
plt.title('Norm of Joint Torques Over Time')
plt.legend()
plt.show()

# =====================================================================
# STEP 4: Replay motion in Meshcat
# =====================================================================
print("\nReplaying motion in Meshcat...")
print("Press Ctrl+C to stop the animation")

try:
    # Play the motion multiple times
    for loop in range(3):  # Play 3 times
        print(f"\nLoop {loop + 1}/3")
        for idx, q in enumerate(configurations):
            # Update forward kinematics before displaying
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            viz.display(q)
            # time.sleep(0.01)  # 10ms delay = ~100 Hz playback
            
    
    print("\nAnimation complete! Viewer remains open.")
    print("The browser window will stay open. Close it manually when done.")
    
except KeyboardInterrupt:
    print("\nAnimation stopped by user.")

# =====================================================================
# STEP 5: Cleanup - Explicitly delete Pinocchio objects to prevent
# double-free corruption during garbage collection
# =====================================================================
del viz
del data
del model
print("\nCleanup completed successfully")

