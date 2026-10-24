#!/home/saakee/Research/Saaketh_INEKF/g1_env/bin/python
"""
Meshcat demo: point moving in an outward spiral (circular + radial velocity).
  omega  = 1 rad/s  (tangential, counter-clockwise)
  r_dot  = 0.1 m/s  (radial outward)

Frame X axis always points RADIALLY OUTWARD (tracks body rotation).

Prints frame velocity in WORLD, LOCAL, and LOCAL_WORLD_ALIGNED at every step.

Expected (see derivation in comments below):
  LOCAL  : vx = r_dot = const 0.1,  vy = r*omega (grows),  vz = 0
  LWA    : true world velocity of the point  [vx_w, vy_w]
  WORLD  : LWA + transport term (p × omega) — cancels tangential part

Run:
  cd /home/saakee/Research/Saaketh_INEKF
  /home/saakee/Research/Saaketh_INEKF/g1_env/bin/python Extra_Files/meshcat_ball_velocity_demo.py
"""

import sys
sys.path.append('/opt/openrobots/lib/python3.10/site-packages')

import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import meshcat.geometry as mg
import meshcat.transformations as mt
import time

# ──────────────────────────────────────────────────────────────────────────────
# Build a minimal pinocchio model: universe → free-flyer ("ball")
# ──────────────────────────────────────────────────────────────────────────────
model = pin.Model()
geom_model = pin.GeometryModel()

joint_id = model.addJoint(
    0, pin.JointModelFreeFlyer(),
    pin.SE3.Identity(), "ball_joint"
)

# Frame with NO offset — X axis = body X = radial outward direction
# As the body yaws, LOCAL-X always points away from origin
frame = pin.Frame(
    "ball_frame",
    joint_id,
    0,
    pin.SE3.Identity(),
    pin.FrameType.OP_FRAME
)
FRAME_ID = model.addFrame(frame)

data = model.createData()

print(f"Model: nq={model.nq}, nv={model.nv}")
print(f"Frame ID={FRAME_ID}: '{model.frames[FRAME_ID].name}'")

# ──────────────────────────────────────────────────────────────────────────────
# Meshcat visualizer (geometry-only, no URDF needed)
# ──────────────────────────────────────────────────────────────────────────────
viz = MeshcatVisualizer(model, geom_model, geom_model)
viz.initViewer(open=True)
viz.loadViewerModel()

# Add a red ball directly via the meshcat viewer handle
viz.viewer["ball"].set_object(
    mg.Sphere(0.15),
    mg.MeshLambertMaterial(color=0xff2222)
)

# ── Local axis lines ────────────────────────────────────────────────────────
# Body joint frame: X=red, Y=green, Z=blue  (tracks yaw rotation)
# Ball frame (45° offset):  shown in lighter colours
from meshcat.geometry import Line, PointsGeometry, LineBasicMaterial

L = 0.5  # axis length in metres
O = np.zeros((3, 1), dtype=np.float32)

def make_axis(tip, color):
    pts = np.hstack([O, np.array(tip, dtype=np.float32).reshape(3,1)])
    return Line(PointsGeometry(pts), LineBasicMaterial(color=color, linewidth=5))

# Body joint frame axes
viz.viewer["body_frame/x"].set_object(make_axis([L, 0, 0], 0xff2222))  # red
viz.viewer["body_frame/y"].set_object(make_axis([0, L, 0], 0x22ff22))  # green
viz.viewer["body_frame/z"].set_object(make_axis([0, 0, L], 0x2222ff))  # blue

# # Ball frame axes (45° tilted from body) — lighter colours so you can tell them apart
# viz.viewer["ball_frame_axes/x"].set_object(make_axis([L, 0, 0], 0xff9999))
# viz.viewer["ball_frame_axes/y"].set_object(make_axis([0, L, 0], 0x99ff99))
# viz.viewer["ball_frame_axes/z"].set_object(make_axis([0, 0, L], 0x9999ff))

print("\n" + "=" * 65)
print("MeshCat running at  http://127.0.0.1:7000/static/")
print("=" * 65)
print("\nSpiral motion: omega=1 rad/s, r_dot=0.1 m/s (radial outward)")
print("Body X axis always points RADIALLY OUTWARD\n")
print(f"{'t':>5s} | {'r':>5s} | {'WORLD [vx vy vz]':^36s} | {'LOCAL [vx vy vz]':^36s} | {'LWA [vx vy vz]':^36s}")
print(f"{'':>5s} | {'':>5s} | {'expect: [cos,sin]*sqrt(rdot²+(rω)²)':^36s} | {'expect: [rdot, rω, 0]':^36s} | {'same as LWA':^36s}")
print("-" * 135)

# ──────────────────────────────────────────────────────────────────────────────
# Spiral trajectory: circular (omega=1 rad/s) + radial outward (r_dot=0.1 m/s)
# Body X axis = radial outward direction (body yaws at same rate as theta)
#
# In body frame:
#   v_linear_body = [r_dot, r*omega, 0]   (radial, tangential, 0)
#   v_angular_body = R^T @ [0, 0, omega]  (body yaw rate = omega)
#
# Expected outputs:
#   LOCAL  : vx = 0.1 (const),  vy = r*1.0 (grows),  vz = 0
#   LWA    : world velocity of the point = R_body @ v_body_linear
#   WORLD  : LWA + (p × omega_world)  — transport term cancels tangential
# ──────────────────────────────────────────────────────────────────────────────
OMEGA   = 1.0    # rad/s — angular velocity around world Z
R_DOT   = 0.0   # m/s  — radial outward velocity
R_INIT  = 0.5   # m    — starting radius
DT      = 0.05  # s
N_STEPS = 300
sleep_multiplier = 3.0  # increase to slow down animation for better visualization

q = pin.neutral(model)
v = np.zeros(model.nv)

# Start at (R_INIT, 0, 0), body X pointing in +X (radial at theta=0)
q[0] = R_INIT

theta = 0.0  # current angle
r     = R_INIT

try:
    for step in range(N_STEPS):
        t = step * DT

        # ── Get current rotation ─────────────────────────────────────────────
        R_body = pin.Quaternion(q[6], q[3], q[4], q[5]).toRotationMatrix()

        # ── Set velocity in body frame ───────────────────────────────────────
        # Linear: radial = r_dot along body-X, tangential = r*omega along body-Y
        v[0:3] = np.array([R_DOT, r * OMEGA, 0.0])                  # already in body frame
        # Angular: body yaws at omega (world Z = body Z when flat)
        v[3:6] = R_body.T @ np.array([0.0, 0.0, OMEGA])

        # ── Forward kinematics ───────────────────────────────────────────────
        pin.forwardKinematics(model, data, q, v)
        pin.updateFramePlacements(model, data)
        pin.computeJointJacobians(model, data, q)
        # ── Frame velocities in three reference frames ───────────────────────
        v_world = pin.getFrameVelocity(model, data, FRAME_ID, pin.ReferenceFrame.WORLD).linear
        v_local = pin.getFrameVelocity(model, data, FRAME_ID, pin.ReferenceFrame.LOCAL).linear
        v_lwa   = pin.getFrameVelocity(model, data, FRAME_ID, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED).linear

        # # ── Frame Jacobians ──────────────────────────────────────────────
        # J_lwa   = pin.getFrameJacobian(model, data, FRAME_ID, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        # J_world = pin.getFrameJacobian(model, data, FRAME_ID, pin.ReferenceFrame.WORLD)

        def fmt(arr):
            return f"[{arr[0]:+6.3f}, {arr[1]:+6.3f}, {arr[2]:+6.3f}]"
        def fmt_row(row):
            return "  ".join(f"{v:+7.4f}" for v in row)

        # oMf = SE3 transform from world origin → frame
        # .translation = position of frame origin in world coordinates
        p_world = data.oMf[FRAME_ID].translation

        print(f"{t:5.2f} | {r:5.3f} | {fmt(v_world)} | {fmt(v_local)} | {fmt(v_lwa)}")
        # print(f"  oMf.translation (world pos of frame): {fmt(p_world)}")
        # print(f"  Jacobian LOCAL_WORLD_ALIGNED linear (3x{model.nv}):")
        # for i, label in enumerate(['vx','vy','vz']):
        #     print(f"    {label}: [{fmt_row(J_lwa[i])}]")
        # print(f"  Jacobian WORLD linear (3x{model.nv}):")
        # for i, label in enumerate(['vx','vy','vz']):
        #     print(f"    {label}: [{fmt_row(J_world[i])}]")
        # print()

        # ── Integrate state ──────────────────────────────────────────────────
        r     += R_DOT * DT
        theta += OMEGA * DT
        q[0]   = r * np.cos(theta)
        q[1]   = r * np.sin(theta)
        q[2]   = 0.0

        # Orientation: body X always points radially outward = angle theta from world X
        half   = theta / 2.0
        q[3:7] = [0.0, 0.0, np.sin(half), np.cos(half)]  # [qx, qy, qz, qw]

        # ── Meshcat update ───────────────────────────────────────────────────
        viz.viewer["ball"].set_transform(mt.translation_matrix(q[0:3]))
        viz.viewer["body_frame"].set_transform(data.oMi[joint_id].homogeneous)
        viz.viewer["ball_frame_axes"].set_transform(data.oMf[FRAME_ID].homogeneous)

        time.sleep(DT*sleep_multiplier)

except KeyboardInterrupt:
    pass

print("\n" + "=" * 65)
print("Frame reference summary:")
print("=" * 65)
print("""
  LOCAL  vx = r_dot = const (radial speed along body-X)
         vy = r*omega (tangential, grows with radius)
         → components in body (radial/tangential) axes

  LWA    True world velocity of the point:
           [r_dot*cos(θ) - r*omega*sin(θ),
            r_dot*sin(θ) + r*omega*cos(θ), 0]
         → what InEKF foot velocity should use ✓

  WORLD  LWA + transport term p×ω  (cancels tangential part)
         → only radial component survives
""")

print("Keep the viewer open — press Ctrl+C to exit.")
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass

# /home/saakee/Research/Saaketh_INEKF/g1_env/bin/python Extra_Files/meshcat_ball_velocity_demo.py