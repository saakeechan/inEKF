#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, BoolMultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np
import subprocess
import threading
import time
import csv
import os
import sys
from pathlib import Path
from scipy.spatial.transform import Rotation

# Import C++ estimator bindings
try:
    import inekf_estimator as inekf
    INEKF_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Could not import inekf_estimator module: {e}")
    print("Running in mock mode. Build Python bindings first.")
    INEKF_AVAILABLE = False


class INEKFBridgeNode(Node):
    """
    ROS2 Bridge node for InEKF Estimator
    
    Subscribes to:
        - /imu/data (sensor_msgs/Imu)
        - /joint_states (sensor_msgs/JointState)
        - /contact_states (std_msgs/BoolMultiArray)
    
    Publishes:
        - /inekf/odometry (nav_msgs/Odometry)
        - /inekf/pose (geometry_msgs/PoseStamped)
        - /inekf/velocity (geometry_msgs/TwistStamped)
    """
    
    def __init__(self):
        super().__init__('inekf_bridge_node')
        
        # Parameters
        self.declare_parameter('robot_config', '../config/digit_config.yaml')
        self.declare_parameter('update_rate', 200.0)  # Hz (dt = 0.005)
        self.declare_parameter('cpp_executable', '../build/INEKF_Estimator_Test')
        self.declare_parameter('sensor_data_file', '/tmp/inekf_sensor_data.csv')
        self.declare_parameter('state_output_file', '/tmp/inekf_state_output.csv')
        
        self.robot_config = self.get_parameter('robot_config').value
        self.update_rate = self.get_parameter('update_rate').value
        self.dt = 1.0 / self.update_rate
        self.cpp_executable = self.get_parameter('cpp_executable').value
        self.sensor_file = self.get_parameter('sensor_data_file').value
        self.state_file = self.get_parameter('state_output_file').value
        
        # Data buffers
        self.imu_data = None
        self.joint_data = None
        self.contact_data = None
        self.latest_state = None
        
        self.sensor_lock = threading.Lock()
        self.state_lock = threading.Lock()
        
        # Initialize sensor vector (30 elements based on main.cpp)
        # [0-2]: gyro, [3-5]: accel, [6-]: encoders
        self.sensor_vector = np.zeros(30)
        self.contact_vector = [False, False]  # 2 legs for Digit
        
        # Initialize C++ estimator
        self.cpp_estimator = None
        if INEKF_AVAILABLE:
            self.initialize_cpp_estimator()
        else:
            self.get_logger().warn('Running in MOCK mode - no C++ estimator available')
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
        
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10)
        
        self.contact_sub = self.create_subscription(
            BoolMultiArray,
            '/contact_states',
            self.contact_callback,
            10)
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/inekf/odometry', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/inekf/pose', 10)
        self.vel_pub = self.create_publisher(TwistStamped, '/inekf/velocity', 10)
        
        # Timer for running estimator
        self.timer = self.create_timer(self.dt, self.estimator_step)
        self.get_logger().info(f'C++ Estimator: {"ENABLED" if INEKF_AVAILABLE else "DISABLED (mock mode)"}')
    
    def initialize_cpp_estimator(self):
        """Initialize the C++ InEKF estimator"""
        try:
            self.get_logger().info('Initializing C++ InEKF estimator...')
            
            # Create estimator instance
            self.cpp_estimator = inekf.InEKFilter()
            
            # Load robot configuration
            self.cpp_estimator.load_robot_config(self.robot_config)
            self.get_logger().info('Robot configuration loaded')
            
            # Set estimator parameters
            self.cpp_estimator.slip_rejection_mode = True
            self.cpp_estimator.slip_threshold = 0.3
            self.cpp_estimator.variable_contact_cov_mode = False
            self.cpp_estimator.cov_amplifier = 5.0
            
            # Prepare covariance parameters (12x1 vector)
            covariances = np.array([
                1e-5,   # gyro
                0.1,    # accel
                0.1,    # slip
                1e-4,   # contact
                1e-8,   # encoder
                1e-10,  # bias gyro
                1e-7,   # bias accel
                1e-10,  # prior ori
                1e-10,  # prior vel
                1e-10,  # prior pos
                1e-10,  # prior bias gyro
                1e-10   # prior bias accel
            ])
            
            # Initial condition (16x1 vector)
            # [Px, Py, Pz, qw, qx, qy, qz (not used here), Vx, Vy, Vz, bgx, bgy, bgz, bax, bay, baz]
            initial_condition = np.zeros(16)
            
            # Initialize estimator
            sliding_window_size = 2
            self.cpp_estimator.Initialize(self.dt, covariances, initial_condition, sliding_window_size)
            
            self.get_logger().info('C++ estimator initialized successfully!')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize C++ estimator: {e}')
            self.cpp_estimator = None
        
        # Flag to indicate if estimator is initialized
        self.estimator_ready = False
        
        self.get_logger().info('InEKF Bridge Node initialized')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz (dt={self.dt}s)')
        self.get_logger().info(f'Robot config: {self.robot_config}')
    
    def imu_callback(self, msg):
        """Process IMU data"""
        with self.sensor_lock:
            # Gyroscope (rad/s) - indices 0-2
            self.sensor_vector[0] = msg.angular_velocity.x
            self.sensor_vector[1] = msg.angular_velocity.y
            self.sensor_vector[2] = msg.angular_velocity.z
            
            # Accelerometer (m/s^2) - indices 3-5
            self.sensor_vector[3] = msg.linear_acceleration.x
            self.sensor_vector[4] = msg.linear_acceleration.y
            self.sensor_vector[5] = msg.linear_acceleration.z
            
            self.imu_data = msg
            
            if not self.estimator_ready:
                self.get_logger().info('Received first IMU data', once=True)
    
    def joint_callback(self, msg):
        """Process joint encoder data"""
        with self.sensor_lock:
            # Map joint positions and velocities to sensor vector
            # Note: Actual mapping depends on robot configuration
            # For now, store first N joint positions starting at index 6
            num_joints = min(len(msg.position), 12)  # Limit based on sensor vector size
            for i in range(num_joints):
                if 6 + i < 30:
                    self.sensor_vector[6 + i] = msg.position[i]
            
            # Store joint velocities (if needed, extend sensor vector)
            # For now just log that we received data
            self.joint_data = msg
            using pybind11 bindings
        """
        if not INEKF_AVAILABLE or self.cpp_estimator is None:
            # Return mock state if estimator not available
            state = {
                'position': np.array([0.0, 0.0, 0.85]),
                'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
                'linear_velocity': np.array([0.0, 0.0, 0.0]),
                'angular_velocity': sensor[0:3],
            }
            return state
        
        try:
            # Create RobotStates object to receive output
            robot_state = inekf.RobotStates()
            
            # Run one estimation step
            self.cpp_estimator.Onestep(sensor, contact, robot_state)
            
            # Extract state from C++ output
            # robot_state.X is a 5x5 matrix: [R p; 0 1] with velocity in 4th column
            X = np.array(robot_state.X)
            
            # Extract rotation matrix (top-left 3x3)
            R = X[0:3, 0:3]
            
            # Convert rotation matrix to quaternion
            rot = Rotation.from_matrix(R)
            quat = rot.as_quat()  # Returns [x, y, z, w]
            
            # Extract position (first 3 elements of 5th column)
            position = X[0:3, 4]
            
            # Extract velocity (first 3 elements of 4th column)
            velocity = X[0:3, 3]
            
            # Extract angular velocity from sensor (or could be from state if available)
            angular_velocity = sensor[0:3]
            
            state = {
                'position': position,
                'orientation': quat,
                'linear_velocity': velocity,
                'angular_velocity': angular_velocity,
            }
            
            return state
            
        except Exception as e:
            self.get_logger().error(f'Error running C++ estimator: {e}')
            return Nontep(self):
        """Run one step of the estimator and publish results"""
        if not self.estimator_ready:
            return
        
        with self.sensor_lock:
            sensor_copy = self.sensor_vector.copy()
            contact_copy = self.contact_vector.copy()
        
        # Save sensor data to file for C++ to read
        self.write_sensor_data(sensor_copy, contact_copy)
        
        # Call C++ estimator (in a real implementation, use subprocess or service)
        # For now, just create mock state data
        state = self.run_cpp_estimator(sensor_copy, contact_copy)
        
        # Publish state
        if state is not None:
            self.publish_state(state)
    
    def write_sensor_data(self, sensor, contact):
        """Write sensor data to file for C++ to read"""
        try:
            with open(self.sensor_file, 'w') as f:
                writer = csv.writer(f)
                # Write timestamp, sensor data, contact data
                row = [time.time()] + sensor.tolist() + contact
                writer.writerow(row)
        except Exception as e:
            self.get_logger().error(f'Failed to write sensor data: {e}')
    
    def run_cpp_estimator(self, sensor, contact):
        """
        Run C++ estimator (stub implementation)
        
        In production, you would:
        1. Use subprocess to call C++ executable
        2. Use ROS2 service call to C++ service node
        3. Use shared memory or message passing
        """
        # TODO: Implement actual C++ integration
        # For now, return mock state
        state = {
            'position': np.array([0.0, 0.0, 0.85]),  # x, y, z
            'orientation': np.array([0.0, 0.0, 0.0, 1.0]),  # x, y, z, w (quaternion)
            'linear_velocity': np.array([0.0, 0.0, 0.0]),
            'angular_velocity': sensor[0:3],  # Use gyro as angular velocity
        }
        
        return state
    
    def publish_state(self, state):
        """Publish estimated state to ROS topics"""
        timestamp = self.get_clock().now().to_msg()
        
        # Publish Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = 'world'
        odom_msg.child_frame_id = 'base_link'
        
        odom_msg.pose.pose.position.x = state['position'][0]
        odom_msg.pose.pose.position.y = state['position'][1]
        odom_msg.pose.pose.position.z = state['position'][2]
        
        odom_msg.pose.pose.orientation.x = state['orientation'][0]
        odom_msg.pose.pose.orientation.y = state['orientation'][1]
        odom_msg.pose.pose.orientation.z = state['orientation'][2]
        odom_msg.pose.pose.orientation.w = state['orientation'][3]
        
        odom_msg.twist.twist.linear.x = state['linear_velocity'][0]
        odom_msg.twist.twist.linear.y = state['linear_velocity'][1]
        odom_msg.twist.twist.linear.z = state['linear_velocity'][2]
        
        odom_msg.twist.twist.angular.x = state['angular_velocity'][0]
        odom_msg.twist.twist.angular.y = state['angular_velocity'][1]
        odom_msg.twist.twist.angular.z = state['angular_velocity'][2]
        
        self.odom_pub.publish(odom_msg)
        
        # Publish Pose
        pose_msg = PoseStamped()
        pose_msg.header = odom_msg.header
        pose_msg.pose = odom_msg.pose.pose
        self.pose_pub.publish(pose_msg)
        
        # Publish Velocity
        vel_msg = TwistStamped()
        vel_msg.header = odom_msg.header
        vel_msg.twist = odom_msg.twist.twist
        self.vel_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = INEKFBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
