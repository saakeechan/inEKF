# Robot Model Configuration Guide

This guide explains how to configure and switch between different robot models in the Invariant Smoother framework.

## Current Architecture

The system is now organized so users primarily interact with two files:
1. **main.cpp** - For experiment parameters and configuration
2. **InEKF_estimator.hpp** - For selecting which robot model to use

## Switching Robot Models

### Step 1: Select Robot Model in InEKF_estimator.hpp

Open `src/Z1_estimation/estimator/InEKF_estimator.hpp` and locate the robot model includes section (around line 10-15):

```cpp
// Include the robot-specific model here
// To use a different robot, change this include to the appropriate robot model
// e.g., #include "../utility/Robot_Model_Cassie.hpp"
#include "../utility/Robot_Model_Digit.hpp"

// Type alias for the robot model (makes it easy to switch robots)
using Robot_Model = Robot_Model_Digit;
```

Change both the include and the type alias to your desired robot:
- For Digit: `#include "../utility/Robot_Model_Digit.hpp"` and `using Robot_Model = Robot_Model_Digit;`
- For other robots: Create a new robot model file and include it here

### Step 2: Configure Parameters in main.cpp

Open `src/main.cpp` and modify the configuration section at the top:

```cpp
// SLIDING WINDOW SIZE:
// This controls the buffer size for the smoother/filter
// Must match SLIDING_WINDOW_SIZE in the robot model header
constexpr int SLIDING_WINDOW_SIZE = 2;

// EXPERIMENT PARAMETERS:
// Configure your experiment parameters below
```

### Step 3: Rebuild the Project

```bash
cd build
cmake ..
make
```

## Creating a New Robot Model

To add support for a new robot, follow these steps:

### 1. Create Robot Model Header

Create `src/Z1_estimation/utility/Robot_Model_<YourRobot>.hpp`:

```cpp
#ifndef SRC_ROBOT_MODEL_YOURROBOT_HPP
#define SRC_ROBOT_MODEL_YOURROBOT_HPP

// Include necessary headers (copy from Robot_Model_Digit.hpp)

class Robot_Model_YourRobot {
public:
    Robot_Model_YourRobot();
    ~Robot_Model_YourRobot();
    
    // Configuration constants
    static constexpr int NUM_LEGS = 4;  // or 2 for biped
    static constexpr int NUM_CONTACTS = NUM_LEGS;
    static constexpr int SLIDING_WINDOW_SIZE = 2;
    
    // Robot model file path
    std::string robot_model_path = "path/to/your/robot.xml";
    
    // Contact point frame IDs (get these from your URDF/MJCF)
    std::vector<int> contact_point_frame_ids = {id1, id2, id3, id4};
    
    // Encoder dimensions
    static constexpr int ENCODER_SIZE = 61;     // nq from Pinocchio
    static constexpr int ENCODERDOT_SIZE = 54;  // nv from Pinocchio
    
    // Copy remaining member variables and functions from Robot_Model_Digit.hpp
    // ...
};

#endif
```

### 2. Create Robot Model Implementation

Create `src/Z1_estimation/utility/Robot_Model_<YourRobot>.cpp`:
- Copy structure from `Robot_Model_Digit.cpp`
- Implement constructor, destructor, and member functions
- The kinematics functions (Forward_Kinematics, get_foot_position, etc.) typically don't need changes as they use Pinocchio

### 3. Update Encoder Assignment in InEKF_estimator.cpp

In `src/Z1_estimation/estimator/InEKF_estimator.cpp`, locate the `new_measurement` function.

Find the encoder assignment section (marked with "ROBOT-SPECIFIC ENCODER ASSIGNMENT" comment) and update it to match your robot's joint structure:

```cpp
// Map your CSV data to encoder indices
ENCODER[frame_count](0) = csvData.joint1[j];
ENCODER[frame_count](1) = csvData.joint2[j];
// ... etc for all joints

// Map your CSV data to encoder velocity indices  
ENCODERDOT[frame_count](0) = csvData.joint1_vel[j];
ENCODERDOT[frame_count](1) = csvData.joint2_vel[j];
// ... etc for all joint velocities
```

**Important**: The encoder indices must match your robot's URDF/MJCF joint ordering in Pinocchio.

### 4. Update CSV Parser (if needed)

If your robot uses different sensor data format:
- Create new CSV parser similar to `csvParserDigit.cpp`
- Update `DigitCSVData.hpp` or create new data structure
- Update `LoadCSVData` function in InEKF_estimator.cpp

## Robot Model Details

### Digit Robot (Current Default)

- **Type**: Bipedal humanoid
- **Legs**: 2
- **Contact frames**: Left foot (31), Right foot (71)
- **Configuration dimension (nq)**: 61
  - 7 for base (3 position + 4 quaternion)
  - 54 joint positions
- **Velocity dimension (nv)**: 54
  - 6 for base (3 linear + 3 angular)
  - 48 joint velocities
- **Model file**: `digit_mujoco-main/src/digit_mujoco/models/flat/digit-v3-flat.xml`

### Adding More Robots

We encourage users to create robot model files for:
- Cassie
- Mini Cheetah
- Spot
- ANYmal
- Other bipeds/quadrupeds

Please follow the structure of `Robot_Model_Digit.hpp` for consistency.

## File Organization

```
src/
├── main.cpp                              # Main configuration (user edits)
└── Z1_estimation/
    ├── estimator/
    │   ├── InEKF_estimator.hpp          # Robot selection (user edits)
    │   ├── InEKF_estimator.cpp          # Encoder assignment (robot-specific)
    │   └── ...
    └── utility/
        ├── Robot_Model_Digit.hpp        # Digit robot configuration
        ├── Robot_Model_Digit.cpp        # Digit robot implementation
        ├── Robot_Model.hpp              # Legacy generic model (deprecated)
        └── Robot_Model.cpp              # Legacy generic model (deprecated)
```

## FAQ

**Q: Do I need to modify CMakeLists.txt when adding a new robot?**  
A: No, it uses glob patterns that automatically include new .cpp files in the utility folder.

**Q: How do I find the contact frame IDs for my robot?**  
A: Load your robot model in Pinocchio and print `model.nframes` and iterate through frames to find the contact points (feet, end-effectors, etc.).

**Q: What if my robot has a different number of legs?**  
A: Update `NUM_LEGS` and `NUM_CONTACTS` in your robot model header, and adjust the contact frame IDs accordingly.

**Q: Can I use URDF instead of MJCF?**  
A: Yes, in your robot model's `load_xml` function, replace `pinocchio::mjcf::buildModel` with `pinocchio::urdf::buildModel`.

## Support

For questions or issues with robot model configuration, please check:
1. Pinocchio documentation for your robot model format
2. Example implementations in Robot_Model_Digit.hpp/cpp
3. Comments in InEKF_estimator.cpp for encoder assignment guidance
