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
urdf_path = "unitree_ros/robots/g1_description/g1_29dof_mode_15.urdf"
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
viz.viewer["left_foot_marker"].set_object(g.Sphere(0.01), g.MeshLambertMaterial(color=0xff0000))

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
duration = 8.0  # seconds
fps = 30
dt = 1.0 / fps
num_frames = int(duration * fps)

# Swing motion parameters
swing_freq = 0.5  # Hz (slower for better viewing)
swing_amplitude_hip = 0.6  # radians (~34 degrees)
swing_amplitude_knee = 1.0  # radians (~57 degrees)

print(f"\nStarting animation ({duration}s at {fps} fps)")
print("Press Ctrl+C to stop\n")

# Animation loop
try:
    for frame in range(num_frames):
        t = frame * dt
        phase = 2 * np.pi * swing_freq * t
        
        # Initialize configuration
        q = np.zeros(model.nq)
        
        # Left leg swing (forward-backward)
        hip_angle = swing_amplitude_hip * np.sin(phase)
        knee_angle = swing_amplitude_knee * max(0, np.sin(phase))  # Only flexion
        
        q[LEFT_HIP_PITCH] = hip_angle
        q[LEFT_KNEE] = knee_angle
        q[LEFT_ANKLE_PITCH] = -0.4 * hip_angle  # Counterbalance
        
        # # Right leg swing (opposite phase)
        # hip_angle_right = swing_amplitude_hip * np.sin(phase + np.pi)
        # knee_angle_right = swing_amplitude_knee * max(0, np.sin(phase + np.pi))
        
        # q[RIGHT_HIP_PITCH] = hip_angle_right
        # q[RIGHT_KNEE] = knee_angle_right
        # q[RIGHT_ANKLE_PITCH] = -0.4 * hip_angle_right
        
        # Compute forward kinematics
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        
        # Update visualization
        viz.display(q)
        
        # Update left foot marker position at frame 31
        left_foot_pos = data.oMf[16].translation
        viz.viewer["left_foot_marker"].set_transform(
            tf.translation_matrix(left_foot_pos)
        )
        
        # Progress indicator
        if frame % fps == 0:
            progress = (frame / num_frames) * 100
            print(f"Progress: {progress:.0f}% | Time: {t:.1f}s / {duration:.1f}s | "
                  f"Left Hip: {np.rad2deg(q[LEFT_HIP_PITCH]):+6.1f}° | "
                  f"Right Hip: {np.rad2deg(q[RIGHT_HIP_PITCH]):+6.1f}°")
        
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
