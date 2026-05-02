"""
Contact-Constrained Factor Graph Smoothing using GTSAM
For legged robot state estimation with IMU and kinematic measurements
"""

import numpy as np
import gtsam
from gtsam import (Point3, Pose3, Rot3,
                   NonlinearFactorGraph, Values, LevenbergMarquardtOptimizer,
                   PriorFactorPose3,
                   noiseModel, symbol)

# Symbol shortcuts for state variables
X = lambda i: symbol('x', i)  # Pose (rotation + position)
V = lambda i: symbol('v', i)  # Velocity
B = lambda i: symbol('b', i)  # IMU bias
L = lambda leg_id, seg_id: symbol('l', leg_id * 100 + seg_id)  # Landmark/contact position for leg


class ContactConstraintFactor(gtsam.CustomFactor):
    """
    Custom factor enforcing zero velocity constraint when foot is in contact
    Constrains velocity at pose X(i) to be near zero
    """
    def __init__(self, pose_key, vel_key, noise_model):
        gtsam.CustomFactor.__init__(self, noise_model, [pose_key, vel_key])
        
    def error(self, this, values, jacobians):
        """Error function: should return velocity (ideally zero when in contact)"""
        velocity = values.atVector(self.keys()[1])
        
        # Error is the velocity itself (should be zero for stationary contact)
        error = velocity
        
        if jacobians is not None:
            # Jacobian w.r.t. pose is zero (velocity doesn't depend on pose directly)
            jacobians[0] = np.zeros((3, 6))
            # Jacobian w.r.t. velocity is identity
            jacobians[1] = np.eye(3)
            
        return error


class KinematicFactor(gtsam.CustomFactor):
    """
    Kinematic constraint factor relating body pose to foot contact position
    Enforces: p_foot_world = R_body * p_foot_body + t_body
    """
    def __init__(self, pose_key, contact_key, foot_position_body, noise_model):
        gtsam.CustomFactor.__init__(self, noise_model, [pose_key, contact_key])
        self.foot_pos_body = foot_position_body  # Foot position in body frame (from FK)
        
    def error(self, this, values, jacobians):
        """Error between predicted foot position and contact landmark"""
        pose = values.atPose3(self.keys()[0])
        contact_pos = values.atPoint3(self.keys()[1])
        
        # Predicted foot position in world frame
        predicted_foot = pose.transformFrom(Point3(self.foot_pos_body))
        
        # Error is difference between predicted and stored contact position
        error = predicted_foot - contact_pos
        
        if jacobians is not None:
            # Jacobian w.r.t. pose (6-DOF)
            R = pose.rotation().matrix()
            p = self.foot_pos_body
            jacobians[0] = np.hstack([
                R,  # Translation part
                -R @ skew(p)  # Rotation part
            ])
            # Jacobian w.r.t. contact position
            jacobians[1] = -np.eye(3)
            
        return error


def skew(v):
    """Skew-symmetric matrix from 3D vector"""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])


def create_contact_constrained_smoother():
    """
    Create a factor graph with IMU preintegration and contact constraints
    """
    
    # Parameters
    num_poses = 20  # Number of time steps
    dt = 0.01  # Time step (10 ms)
    num_legs = 4  # Quadruped robot
    
    # Noise models
    prior_pose_noise = noiseModel.Diagonal.Sigmas(
        np.array([0.1, 0.1, 0.1, 0.3, 0.3, 0.3]))  # rotation (rad), translation (m)
    prior_vel_noise = noiseModel.Isotropic.Sigma(3, 0.1)  # m/s
    prior_bias_noise = noiseModel.Isotropic.Sigma(6, 1e-3)
    
    # Contact constraint noise (very small when foot is in contact)
    contact_noise = noiseModel.Isotropic.Sigma(3, 0.001)  # 1mm std dev
    
    # Kinematic constraint noise
    kinematic_noise = noiseModel.Isotropic.Sigma(3, 0.01)  # 1cm std dev
    
    # IMU noise parameters
    accel_noise_sigma = 0.0003924
    gyro_noise_sigma = 0.000205689024915
    accel_bias_rw_sigma = 0.004905
    gyro_bias_rw_sigma = 0.000001454441043
    
    imu_params = gtsam.PreintegrationParams.MakeSharedU(9.81)
    imu_params.setAccelerometerCovariance(np.eye(3) * accel_noise_sigma**2)
    imu_params.setGyroscopeCovariance(np.eye(3) * gyro_noise_sigma**2)
    imu_params.setIntegrationCovariance(np.eye(3) * 1e-8)
    imu_params.setUse2ndOrderCoriolis(False)
    imu_params.setOmegaCoriolis(np.zeros(3))
    
    # Create factor graph and initial values
    graph = NonlinearFactorGraph()
    initial_values = Values()
    
    # Initial state
    initial_pose = Pose3(Rot3.Ypr(0, 0, 0), Point3(0, 0, 0.3))  # 30cm above ground
    initial_velocity = np.array([0.0, 0.0, 0.0])
    initial_bias = gtsam.imuBias.ConstantBias()  # Zero bias initially
    
    # Add prior factors for initial state
    graph.add(PriorFactorPose3(X(0), initial_pose, prior_pose_noise))
    graph.add(gtsam.PriorFactorVector(V(0), initial_velocity, prior_vel_noise))
    graph.add(gtsam.PriorFactorConstantBias(B(0), initial_bias, prior_bias_noise))
    
    # Insert initial values
    initial_values.insert(X(0), initial_pose)
    initial_values.insert(V(0), initial_velocity)
    initial_values.insert(B(0), initial_bias)
    
    # Simulated contact schedule (binary: True = contact, False = swing)
    # Each leg alternates contact pattern
    contact_schedule = []
    for i in range(num_poses):
        contacts = [
            i % 10 < 5,  # Leg 0: contact for first half of cycle
            (i + 5) % 10 < 5,  # Leg 1: opposite phase
            i % 10 < 5,  # Leg 2: same as leg 0 (diagonal pair)
            (i + 5) % 10 < 5,  # Leg 3: same as leg 1
        ]
        contact_schedule.append(contacts)
    
    # Foot positions in body frame (typical quadruped geometry)
    foot_positions_body = np.array([
        [0.2, 0.15, -0.3],   # Front left
        [0.2, -0.15, -0.3],  # Front right
        [-0.2, 0.15, -0.3],  # Rear left
        [-0.2, -0.15, -0.3], # Rear right
    ])
    
    # Store contact landmark positions (world frame, initialized when contact made)
    contact_landmarks = {}  # Key: (leg_id, contact_segment_id) -> landmark key
    current_contact_segment = [0] * num_legs  # Track which contact segment each leg is in
    
    # Build factor graph
    for i in range(1, num_poses):
        # Simulate IMU measurements (simple forward motion)
        measured_accel = np.array([0.1, 0.0, 9.81])  # Slight forward acceleration + gravity
        measured_omega = np.array([0.0, 0.0, 0.05])  # Slight yaw rate
        
        # Create IMU preintegration
        pim = gtsam.PreintegratedImuMeasurements(imu_params, initial_bias)
        pim.integrateMeasurement(measured_accel, measured_omega, dt)
        
        # Add IMU factor between consecutive poses
        imu_factor = gtsam.ImuFactor(
            X(i-1), V(i-1),
            X(i), V(i),
            B(i-1),
            pim
        )
        graph.add(imu_factor)
        
        # Add bias evolution factor (bias is nearly constant)
        bias_noise = noiseModel.Isotropic.Sigma(6, np.sqrt(dt) * 1e-3)
        graph.add(gtsam.BetweenFactorConstantBias(B(i-1), B(i), 
                                                   gtsam.imuBias.ConstantBias(),
                                                   bias_noise))
        
        # Initialize current state (rough estimate for optimization)
        # Simple dead reckoning for initialization
        prev_pose = initial_values.atPose3(X(i-1)) if i == 1 else initial_values.atPose3(X(i-1))
        new_pose = prev_pose.compose(Pose3(Rot3(), Point3(0.01, 0, 0)))  # Move forward
        initial_values.insert(X(i), new_pose)
        initial_values.insert(V(i), np.array([0.1, 0.0, 0.0]))
        initial_values.insert(B(i), initial_bias)
        
        # Process each leg for contact constraints
        for leg_id in range(num_legs):
            in_contact = contact_schedule[i][leg_id]
            was_in_contact = contact_schedule[i-1][leg_id]
            
            if in_contact:
                # Check if this is start of new contact
                if not was_in_contact:
                    current_contact_segment[leg_id] += 1
                
                segment_id = current_contact_segment[leg_id]
                landmark_key = L(leg_id, segment_id)
                
                # If first time seeing this contact segment, initialize landmark
                if (leg_id, segment_id) not in contact_landmarks:
                    # Compute initial foot position in world frame
                    pose = initial_values.atPose3(X(i))
                    foot_world = pose.transformFrom(Point3(foot_positions_body[leg_id]))
                    initial_values.insert(landmark_key, foot_world)
                    contact_landmarks[(leg_id, segment_id)] = landmark_key
                
                # Add contact constraint: zero velocity when foot is on ground
                # Using a simple velocity prior rather than custom factor for simplicity
                graph.add(gtsam.PriorFactorVector(V(i), np.array([0.0, 0.0, 0.0]), contact_noise))
                
                # Add kinematic factor: body pose -> foot position
                # This constrains the foot to stay at the contact landmark position
                # Using a simple Between factor approximation
                # For production: use custom KinematicFactor class defined above
                foot_body = foot_positions_body[leg_id]
                pose_at_i = initial_values.atPose3(X(i))
                foot_world_current = pose_at_i.transformFrom(Point3(foot_body))
                
                # Point-to-point constraint between foot and contact landmark
                # graph.add(KinematicFactor(X(i), landmark_key, foot_body, kinematic_noise))
                # Simplified: Add a unary factor on the contact landmark
                graph.add(gtsam.PriorFactorPoint3(landmark_key, foot_world_current, 
                                                  kinematic_noise))
    
    return graph, initial_values, contact_schedule


def optimize_and_visualize(graph, initial_values):
    """Optimize the factor graph"""
    
    print(f"Factor graph has {graph.size()} factors")
    print(f"Initial error: {graph.error(initial_values):.6f}")
    
    # Set up optimizer
    params = gtsam.LevenbergMarquardtParams()
    params.setVerbosityLM("SUMMARY")
    params.setMaxIterations(100)
    params.setRelativeErrorTol(1e-5)
    params.setAbsoluteErrorTol(1e-5)
    
    # Optimize
    optimizer = LevenbergMarquardtOptimizer(graph, initial_values, params)
    result = optimizer.optimize()
    
    print(f"Final error: {graph.error(result):.6f}")
    print(f"Optimization complete!")
    
    return result


def extract_trajectory(result, num_poses):
    """Extract optimized trajectory"""
    trajectory = []
    velocities = []
    
    for i in range(num_poses):
        pose = result.atPose3(X(i))
        vel = result.atVector(V(i))
        
        position = pose.translation()
        rotation = pose.rotation().rpy()
        
        trajectory.append([position[0], position[1], position[2]])
        velocities.append([vel[0], vel[1], vel[2]])
    
    return np.array(trajectory), np.array(velocities)


def main():
    """Main execution"""
    print("="*60)
    print("Contact-Constrained Factor Graph Smoothing with GTSAM")
    print("="*60)
    
    # Create factor graph
    print("\n1. Building factor graph...")
    graph, initial_values, contact_schedule = create_contact_constrained_smoother()
    
    # Optimize
    print("\n2. Optimizing...")
    result = optimize_and_visualize(graph, initial_values)
    
    # Extract results
    print("\n3. Extracting trajectory...")
    num_poses = len(contact_schedule)
    trajectory, velocities = extract_trajectory(result, num_poses)
    
    print(f"\nTrajectory shape: {trajectory.shape}")
    print(f"Initial position: {trajectory[0]}")
    print(f"Final position: {trajectory[-1]}")
    print(f"\nMean velocity: {np.mean(velocities, axis=0)}")
    
    # Show contact schedule
    print("\nContact schedule (first 10 steps):")
    print("Time | FL FR RL RR")
    print("-" * 20)
    for i in range(min(10, num_poses)):
        contacts = contact_schedule[i]
        contact_str = ''.join(['X' if c else 'O' for c in contacts])
        print(f"{i:4d} | {' '.join(contact_str)}")
    
    print("\n" + "="*60)
    print("Smoothing complete!")
    print("="*60)


if __name__ == "__main__":
    main()