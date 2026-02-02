#!/usr/bin/env python3
"""
Go2 Robot Leg Swing Animation using Pinocchio and MeshCat
Displays 3D animated robot with swinging legs in browser
"""

import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import numpy as np
import time

# Load Go2 URDF model
urdf_path = "unitree_ros/robots/go2_description/urdf/go2_description.urdf"
# For package://go2_description/dae/... to resolve correctly
mesh_dir = "unitree_ros/robots"

# Frame index for front left foot marker (manually set this)
front_left_foot_frame = 15

print(f"Loading Go2 model from: {urdf_path}")

# Load model with visual geometry
try:
    model, collision_model, visual_model = pin.buildModelsFromUrdf(urdf_path, mesh_dir)
    has_visual = True
except ValueError as e:
    print(f"Warning: Could not load visual models: {e}")
    print("Loading model without visual geometry...")
    model = pin.buildModelFromUrdf(urdf_path)
    collision_model = None
    visual_model = None
    has_visual = False

data = model.createData()

print(f"\nModel loaded successfully!")
print(f"Number of joints (nq): {model.nq}")
print(f"Number of velocities (nv): {model.nv}")

# Initialize MeshCat visualizer
print("\nInitializing MeshCat visualizer...")
if has_visual:
    viz = MeshcatVisualizer(model, collision_model, visual_model)
else:
    viz = MeshcatVisualizer(model, pin.GeometryModel(), pin.GeometryModel())

try:
    viz.initViewer(open=True)
except ImportError as err:
    print("Error while initializing the viewer. It seems you should install python meshcat")
    print(err)
    import sys
    sys.exit(0)

try:
    viz.loadViewerModel(rootNodeName="go2_robot")
    print("Robot model loaded in MeshCat viewer!")
except Exception as e:
    print(f"Warning: {e}")
    print("Continuing anyway...")

# Add red sphere marker at front left foot
import meshcat.geometry as g
import meshcat.transformations as tf
viz.viewer["front_left_foot_marker"].set_object(g.Sphere(0.005), g.MeshLambertMaterial(color=0xff0000))

print(f"\nUsing frame index {front_left_foot_frame} for foot marker")
print(f"Frame name: {model.frames[front_left_foot_frame].name}")

print("\n" + "="*60)
print("MeshCat viewer is now running!")
print("Open your browser to view the animation")
print("URL: http://127.0.0.1:7000/static/")
print("="*60)

# Joint indices for leg control (quadruped configuration)
# Front Left
FL_HIP = 0
FL_THIGH = 1
FL_CALF = 2

# Front Right
FR_HIP = 3
FR_THIGH = 4
FR_CALF = 5

# Rear Left
RL_HIP = 6
RL_THIGH = 7
RL_CALF = 8

# Rear Right
RR_HIP = 9
RR_THIGH = 10
RR_CALF = 11

# Animation parameters
duration = 8.0  # seconds
fps = 30
dt = 1.0 / fps
num_frames = int(duration * fps)

# Swing motion parameters
swing_freq = 0.5  # Hz (slower for better viewing)
swing_amplitude_thigh = 0.6  # radians (~34 degrees)
swing_amplitude_calf = 1.0  # radians (~57 degrees)

print(f"\nStarting static display ({duration}s)")
print("Press Ctrl+C to stop\n")

# Static display loop
try:
    # Initialize configuration - all joints at zero (neutral pose)
    q = np.zeros(model.nq)
    
    # Set static pose angles (optional - uncomment and adjust if desired)
    # q[FL_THIGH] = 0.0
    # q[FL_CALF] = 0.0
    
    # Compute forward kinematics once
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    
    # Display the static pose
    viz.display(q)
    
    # Update front left foot marker position
    foot_pos = data.oMf[front_left_foot_frame].translation
    viz.viewer["front_left_foot_marker"].set_transform(
        tf.translation_matrix(foot_pos)
    )
    
    print(f"Robot displayed in static pose (all joints at neutral/zero position)")
    
    # Keep display loop simple
    for frame in range(num_frames):
        t = frame * dt
        
        # Progress indicator (every second)
        if frame % fps == 0:
            progress = (frame / num_frames) * 100
            print(f"Display time: {t:.1f}s / {duration:.1f}s ({progress:.0f}%)")
        
        time.sleep(dt)
    
    print("\n" + "="*60)
    print("Static display complete!")
    print("Viewer will stay open. Press Ctrl+C to exit.")
    print("="*60)
    
    # Keep viewer open
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("\n\nAnimation stopped by user.")
    print("Closing viewer...")

print("\nDone!")
