#!/usr/bin/env python3
"""
G1 Robot Leg Swing Animation using Pinocchio and MeshCat
Displays 3D animated robot with swinging legs in browser
"""

import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import numpy as np
import time

# Load G1 URDF model
urdf_path = "/home/saakee/Research/Saaketh_INEKF/src/utility/G1/g1_29dof_IMUShifted.urdf"
mesh_dir = "unitree_ros/robots/g1_description"

print(f"Loading G1 model from: {urdf_path}")

# Load model with visual geometry
model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_path, mesh_dir)
data = model.createData()

print(f"\nModel loaded successfully!")
print(f"Number of joints (nq): {model.nq}")
print(f"Number of velocities (nv): {model.nv}")

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

# Add red sphere marker at left foot (frame 15)
import meshcat.geometry as g
import meshcat.transformations as tf
viz.viewer["left_foot_marker"].set_object(g.Sphere(0.075), g.MeshLambertMaterial(color=0xff0000))

print("\n" + "="*60)
print("MeshCat viewer is now running!")
print("Open your browser to view the animation")
print("URL: http://127.0.0.1:7000/static/")
print("="*60)

# Joint indices for leg control
LEFT_HIP_PITCH = 0
LEFT_HIP_ROLL = 1
LEFT_HIP_YAW = 2
LEFT_KNEE = 3
LEFT_ANKLE_PITCH = 4
LEFT_ANKLE_ROLL = 5

RIGHT_HIP_PITCH = 6
RIGHT_HIP_ROLL = 7
RIGHT_HIP_YAW = 8
RIGHT_KNEE = 9
RIGHT_ANKLE_PITCH = 10
RIGHT_ANKLE_ROLL = 11

# Animation parameters
duration = 5.0  # seconds
fps = 30
dt = 1.0 / fps
num_frames = int(duration * fps)

# Swing motion parameters
swing_freq = 0.5  # Hz (slower for better viewing)
swing_amplitude_hip = 0.6  # radians (~34 degrees)
swing_amplitude_knee = 1.0  # radians (~57 degrees)

print(f"\nStarting animation ({duration}s at {fps} fps)")
print("Press Ctrl+C to stop\n")

# Generate random frequencies and amplitudes for each joint
np.random.seed(42)  # For reproducibility
joint_frequencies = np.random.uniform(0.3, 1.0, model.nq)  # Hz
joint_amplitudes = np.random.uniform(0.2, 0.8, model.nq)  # radians
joint_phases = np.random.uniform(0, 2*np.pi, model.nq)  # random phase offset

print(f"Animating all {model.nq} joints with random motions")

# Specify the frame index to track velocity (frame 1 - left foot)
FRAME_ID = 4

# Animation loop
try:
    for frame in range(num_frames):
        t = frame * dt
        
        # Initialize configuration, velocity, and acceleration
        q = np.zeros(model.nq)
        v = np.zeros(model.nv)
        a = np.zeros(model.nv)
        
        # Apply random sinusoidal motion to all joints
        for joint_idx in range(model.nq):
            phase = 2 * np.pi * joint_frequencies[joint_idx] * t + joint_phases[joint_idx]
            omega = joint_frequencies[joint_idx] * 2 * np.pi
            q[joint_idx] = joint_amplitudes[joint_idx] * np.sin(phase)
            # Compute joint velocity (derivative of position)
            v[joint_idx] = joint_amplitudes[joint_idx] * omega * np.cos(phase)
            # Compute joint acceleration (derivative of velocity)
            a[joint_idx] = -joint_amplitudes[joint_idx] * omega * omega * np.sin(phase)
        
        # Compute forward kinematics with velocities and accelerations
        pin.forwardKinematics(model, data, q, v, a)
        pin.updateFramePlacements(model, data)
        
        # # Compute frame velocity and acceleration (print every 10 frames)
        # if frame % 10 == 0:
        #     frame_velocity = pin.getFrameVelocity(model, data, FRAME_ID, pin.ReferenceFrame.WORLD)
        #     frame_acceleration = pin.getFrameAcceleration(model, data, FRAME_ID, pin.ReferenceFrame.WORLD)
        #     print(f"Frame {FRAME_ID} | Velocity: linear=[{frame_velocity.linear[0]:.4f}, {frame_velocity.linear[1]:.4f}, {frame_velocity.linear[2]:.4f}], "
        #           f"angular=[{frame_velocity.angular[0]:.4f}, {frame_velocity.angular[1]:.4f}, {frame_velocity.angular[2]:.4f}]")
        #     print(f"Frame {FRAME_ID} | Acceleration: linear=[{frame_acceleration.linear[0]:.4f}, {frame_acceleration.linear[1]:.4f}, {frame_acceleration.linear[2]:.4f}], "
        #           f"angular=[{frame_acceleration.angular[0]:.4f}, {frame_acceleration.angular[1]:.4f}, {frame_acceleration.angular[2]:.4f}]\n")
        
        # Update visualization
        viz.display(q)
        
        # Update left foot marker position at frame 31
        left_foot_pos = data.oMf[FRAME_ID].translation
        print("Frame: ", left_foot_pos)
        viz.viewer["left_foot_marker"].set_transform(
            tf.translation_matrix(left_foot_pos)
        )

        
        time.sleep(dt)
    
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

# Left foot position:  [ 0.04525  0.      -0.08339] frame 3 - imu pelvis
# Left foot position:  [ 0.04525  0.      -0.08339] frame 2
# Left foot position:  [-0.04313164  0.11850645 -0.78649488] - frame 16 and 17 are the same for torso

# cd /home/saakee/Research/Saaketh_INEKF && /home/saakee/Research/Saaketh_INEKF/g1_env/bin/python Extra_Files/g1_leg_swing_meshcat.py