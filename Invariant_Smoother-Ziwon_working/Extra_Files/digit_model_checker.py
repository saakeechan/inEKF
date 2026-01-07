#!/usr/bin/env python3

import csv
from turtle import right
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import numpy as np
import os
# Import all CSV data variables from the data loader module
from digit_data_loader import *

# Load the Digit robot MuJoCo model
mujoco_path = os.path.join(os.path.dirname(__file__), 
                           "digit_mujoco-main/src/digit_mujoco/models/flat/digit-v3-flat.xml")




# Create the robot model from MuJoCo XML with floating base
try:
    model, collision_model, visual_model = pin.buildModelsFromMJCF(
        mujoco_path,
        verbose=False
    )
    data = model.createData()
    
except Exception as e:
    print(f"✗ Error loading MuJoCo model: {e}")
    import sys
    sys.exit(1)


# Test forward kinematics with zero configuration
q = pin.neutral(model)
pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)

# Print model dimensions and joint information
print(f"\n{'='*60}")
print(f"Model Dimensions:")
print(f"{'='*60}")
print(f"model.nq (configuration space dim): {model.nq}")
print(f"model.nv (velocity space dim): {model.nv}")
print(f"Number of joints: {model.njoints}")
print(f"Number of frames: {model.nframes}")

print(f"\n{'='*60}")
print(f"Joint Name Mapping (nq vs nv indices):")
print(f"{'='*60}")
print(f"{'Joint ID':<10} {'Joint Name':<30} {'nq_idx':<15} {'nv_idx':<15} {'nq_size':<10} {'nv_size':<10}")
print(f"{'-'*100}")

for i, joint in enumerate(model.joints):
    nq_idx = model.joints[i].idx_q
    nv_idx = model.joints[i].idx_v
    nq_size = model.joints[i].nq
    nv_size = model.joints[i].nv
    print(f"{i:<10} {model.names[i]:<30} {nq_idx:<15} {nv_idx:<15} {nq_size:<10} {nv_size:<10}")

print(f"{'='*60}\n")

# Find left foot heel frame
left_foot_heel_idx = None
for idx, frame in enumerate(model.frames):
    if 'left' in frame.name.lower() and 'heel' in frame.name.lower():
        left_foot_heel_idx = idx
        print(f"Found left foot heel frame: {frame.name} (index {idx})")
        break

if left_foot_heel_idx is None:
    print("Warning: Could not find left foot heel frame!")









try:
    import meshcat
    
    # Create viewer with geometry models
    viz = MeshcatVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=False)
    viz.loadViewerModel()
    viz.display(q)
    
    url = viz.viewer.url()
    print(f"\n{'='*60}")
    print(f"  🌐 Open your browser to: {url}")
    print(f"{'='*60}")
    print("\n  The server is running and waiting for connections...")
    print("  Model displayed with LEFT LEG ONLY motion.")
    print("\n  Press Ctrl+C to exit when done viewing...\n")
    
    import time
    import meshcat.geometry as g
    import meshcat.transformations as tf

    sphereFrame = 31  #right foot fixed
    
    # Visualize frame as coordinate axes
    frame_name = model.frames[sphereFrame].name
    print(f"Frame {sphereFrame} is: {frame_name}")
    
    # Create a red sphere at frame location
    sphere_radius = 0.01
    viz.viewer["frame_sphere"].set_object(
        g.Sphere(sphere_radius),
        g.MeshLambertMaterial(color=0xff0000)
    )

    
    # Animate with data from CSV
    i = 0
    dt = 0.005  # 5ms timestep
    q_animated = q.copy()
    q_d_animated = np.zeros(model.nv)  # Joint velocities (nv = number of velocity dimensions)

    try:
        while i < len(timestamp):
            # Start with neutral configuration
            q_animated = q.copy()

            # Set base pose from CSV data
            q_animated[0:7] = pos_x[i], pos_y[i], pos_z[i] - 1.063*1.4, quat_i[i], quat_j[i], quat_k[i], quat_w[i] # figure out z offset later
            
            # Left leg joints
            q_animated[7] = left_hip_roll[i]
            q_animated[8] = left_hip_yaw[i]
            q_animated[9] = left_hip_pitch[i]
            # q_animated[10:14] = left_achilles_rod (spherical - needs quaternion)
            q_animated[14] = left_knee[i]
            q_animated[15] = left_shin[i]
            q_animated[16] = left_tarsus[i]
            q_animated[17] = left_heel_spring[i]
            q_animated[18] = left_toe_A[i]
            # q_animated[19:23] = left_toe_A_rod (spherical - needs quaternion)
            q_animated[23] = left_toe_B[i]
            # q_animated[24:28] = left_toe_B_rod (spherical - needs quaternion)
            q_animated[28] = left_toe_pitch[i]
            q_animated[29] = left_toe_roll[i]
            
            # # Left arm joints
            # q_animated[30] = left_shoulder_roll[i]
            # q_animated[31] = left_shoulder_pitch[i]
            # q_animated[32] = left_shoulder_yaw[i]
            # q_animated[33] = left_elbow[i]
            
            # Right leg joints
            q_animated[34] = right_hip_roll[i]
            q_animated[35] = right_hip_yaw[i]
            q_animated[36] = right_hip_pitch[i]
            # q_animated[37:41] = right_achilles_rod (spherical - needs quaternion)
            q_animated[41] = right_knee[i]
            q_animated[42] = right_shin[i]
            q_animated[43] = right_tarsus[i]
            q_animated[44] = right_heel_spring[i]
            q_animated[45] = right_toe_A[i]
            # q_animated[46:50] = right_toe_A_rod (spherical - needs quaternion)
            q_animated[50] = right_toe_B[i]
            # q_animated[51:55] = right_toe_B_rod (spherical - needs quaternion)
            q_animated[55] = right_toe_pitch[i]
            q_animated[56] = right_toe_roll[i]
            
            # # Right arm joints
            # q_animated[57] = right_shoulder_roll[i]
            # q_animated[58] = right_shoulder_pitch[i]
            # q_animated[59] = right_shoulder_yaw[i]
            # q_animated[60] = right_elbow[i]

            # Set joint velocities from CSV data
            # Base velocities (linear and angular)
            # q_d_animated[0:3] = vel_x[i], vel_y[i], vel_z[i]
            # q_d_animated[3:6] = r_rate[i], p_rate[i], y_rate[i]
            
            # Left leg joint velocities
            q_d_animated[6] = left_hip_roll_vel[i]
            q_d_animated[7] = left_hip_yaw_vel[i]
            q_d_animated[8] = left_hip_pitch_vel[i]
            # q_d_animated[9:12] = 0, 0, 0  # left_achilles_rod (spherical - needs 3D angular velocity)
            q_d_animated[12] = left_knee_vel[i]
            q_d_animated[13] = left_shin_vel[i]
            q_d_animated[14] = left_tarsus_vel[i]
            q_d_animated[15] = left_heel_spring_vel[i]
            q_d_animated[16] = left_toe_A_vel[i]
            # q_d_animated[17:20] = 0, 0, 0  # left_toe_A_rod (spherical - needs 3D angular velocity)
            q_d_animated[20] = left_toe_B_vel[i]
            # q_d_animated[21:24] = 0, 0, 0  # left_toe_B_rod (spherical - needs 3D angular velocity)
            q_d_animated[24] = left_toe_pitch_vel[i]
            q_d_animated[25] = left_toe_roll_vel[i]
            
            # # Left arm joint velocities
            # q_d_animated[26] = left_shoulder_roll_vel[i]
            # q_d_animated[27] = left_shoulder_pitch_vel[i]
            # q_d_animated[28] = left_shoulder_yaw_vel[i]
            # q_d_animated[29] = left_elbow_vel[i]
            
            # Right leg joint velocities
            q_d_animated[30] = right_hip_roll_vel[i]
            q_d_animated[31] = right_hip_yaw_vel[i]
            q_d_animated[32] = right_hip_pitch_vel[i]
            # q_d_animated[33:36] = 0, 0, 0  # right_achilles_rod (spherical - needs 3D angular velocity)
            q_d_animated[36] = right_knee_vel[i]
            q_d_animated[37] = right_shin_vel[i]
            q_d_animated[38] = right_tarsus_vel[i]
            q_d_animated[39] = right_heel_spring_vel[i]
            q_d_animated[40] = right_toe_A_vel[i]
            # q_d_animated[41:44] = 0, 0, 0  # right_toe_A_rod (spherical - needs 3D angular velocity)
            q_d_animated[44] = right_toe_B_vel[i]
            # q_d_animated[45:48] = 0, 0, 0  # right_toe_B_rod (spherical - needs 3D angular velocity)
            q_d_animated[48] = right_toe_pitch_vel[i]
            q_d_animated[49] = right_toe_roll_vel[i]
            
            # # Right arm joint velocities
            # q_d_animated[50] = right_shoulder_roll_vel[i]
            # q_d_animated[51] = right_shoulder_pitch_vel[i]
            # q_d_animated[52] = right_shoulder_yaw_vel[i]
            # q_d_animated[53] = right_elbow_vel[i]

            

            # Compute forward kinematics
            pin.forwardKinematics(model, data, q_animated, q_d_animated)
            pin.updateFramePlacements(model, data)            

            # Get left foot heel position in global frame
            if left_foot_heel_idx is not None:
                # Get left foot heel pose in world frame
                left_heel_world = data.oMf[left_foot_heel_idx]
                left_heel_position = left_heel_world.translation
                
                # # Print every 100 frames to avoid spam
                # if i % 50 == 0:
                #     print(f"Frame {i}: Left heel in global frame: [{left_heel_position[0]:.4f}, {left_heel_position[1]:.4f}, {left_heel_position[2]:.4f}]")

            # for every 10 frames
            if i % 500 == 0:
            
                # Visualize frame - update sphere position
                frame_pose = data.oMf[sphereFrame]
                # print(f"Frame {i}: Frame {sphereFrame} position: {frame_pose.translation}")

                # Get frame velocity in world frame
                frame_velocity = pin.getFrameVelocity(model, data, sphereFrame, pin.WORLD)
                # frame_velocity_body = pin.getFrameVelocity(model, data, 1, pin.WORLD)
                # print(f"Frame {i}: Frame {sphereFrame} velocity: linear={frame_velocity.linear}, angular={frame_velocity.angular}")
                # print(f"Frame {i}: Frame {sphereFrame} velocity (body): linear={frame_velocity_body.linear}, angular={frame_velocity_body.angular}")

                
                # Create transformation matrix with just translation (sphere at frame origin)
                sphere_transform = np.eye(4)
                sphere_transform[:3, 3] = frame_pose.translation
                viz.viewer["frame_sphere"].set_transform(sphere_transform)
                
                # Update visualization
                # viz.display(q_animated)

                # Compute Jacobians for all frames
                pin.computeJointJacobians(model, data, q_animated)


                J_sphere = pin.getFrameJacobian(model, data, sphereFrame, pin.WORLD)
                print(f"Frame {i}: Jacobian vx (first row) of frame {sphereFrame}:\n{J_sphere[0, :]}")


            # Increment index
            i += 1
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n\nViewer closed.")
        
except ImportError as e:
    print("✗ MeshCat viewer not available")
    print(f"  Error: {e}")
    print("  Install with: pip install meshcat")
except Exception as e:
    print(f"✗ Could not launch viewer: {e}")
    import traceback
    traceback.print_exc()
