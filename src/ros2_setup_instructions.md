# ROS2 InEKF Bridge Setup Instructions

## Overview
The `ros2_inekf_bridge.py` node bridges between ROS2 topics and your C++ InEKF estimator.

## Installation

### 1. Install ROS2 Dependencies
```bash
# Make sure ROS2 is installed (e.g., ROS2 Humble)
sudo apt update
sudo apt install python3-colcon-common-extensions
pip3 install numpy
```

### 2. Make the Python script executable
```bash
chmod +x src/ros2_inekf_bridge.py
```

## Usage

### Run the bridge node
```bash
# Source ROS2 workspace
source /opt/ros/humble/setup.bash  # or your ROS2 distro

# Run the node
python3 src/ros2_inekf_bridge.py
```

### With custom parameters
```bash
ros2 run <package_name> ros2_inekf_bridge.py --ros-args \
  -p robot_config:=config/digit_config.yaml \
  -p update_rate:=200.0 \
  -p cpp_executable:=build/INEKF_Estimator_Test
```

## Topic Interface

### Subscribed Topics
- `/imu/data` (sensor_msgs/Imu) - IMU sensor data (gyro + accelerometer)
- `/joint_states` (sensor_msgs/JointState) - Joint encoder positions and velocities
- `/contact_states` (std_msgs/BoolMultiArray) - Contact state for each leg

### Published Topics
- `/inekf/odometry` (nav_msgs/Odometry) - Full state estimate (pose + velocity)
- `/inekf/pose` (geometry_msgs/PoseStamped) - Position and orientation only
- `/inekf/velocity` (geometry_msgs/TwistStamped) - Linear and angular velocity

## Integration Options

### Option A: File-based Communication (Current stub)
- Python writes sensor data to CSV
- C++ reads from CSV and processes
- C++ writes state to output CSV
- Python reads and publishes

### Option B: ROS2 Service (Recommended)
- Convert C++ code to a ROS2 service node
- Python calls service with sensor data
- Service returns estimated state
- Python publishes state

### Option C: Subprocess
- Python spawns C++ executable
- Communicate via stdin/stdout
- Parse output and publish

### Option D: Python Bindings (Best performance)
- Create pybind11 wrapper for C++ estimator
- Import directly in Python
- No IPC overhead

## Next Steps

1. **Choose integration method** (B or D recommended)
2. **Modify C++ code** to support chosen method
3. **Update CMakeLists.txt** to build ROS2 components
4. **Test with sensor data** from real robot or simulator

## Testing

### Test with dummy data
```bash
# Terminal 1: Publish dummy IMU data
ros2 topic pub /imu/data sensor_msgs/msg/Imu "{...}"

# Terminal 2: Publish dummy joint states
ros2 topic pub /joint_states sensor_msgs/msg/JointState "{...}"

# Terminal 3: Publish dummy contact states
ros2 topic pub /contact_states std_msgs/msg/BoolMultiArray "{data: [true, false]}"

# Terminal 4: Run bridge
python3 src/ros2_inekf_bridge.py

# Terminal 5: Monitor output
ros2 topic echo /inekf/odometry
```
