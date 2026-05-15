/* ----------------------------------------------------------------------------
 * Contact Neural Network Estimator
 * Handles neural network-based contact detection functionality
 * Note: NN outputs contact predictions only; foot velocities are computed via kinematics
 * -------------------------------------------------------------------------- */

#ifndef CONTACT_NN_HPP
#define CONTACT_NN_HPP

#include <vector>
#include <deque>
#include <memory>
#include <string>
#include <eigen3/Eigen/Dense>
#include <onnxruntime_cxx_api.h>
#include "../utility/RobotModel.hpp"

class ContactNN {
public:
    // Constants for NN input shape
    static constexpr int NN_WINDOW_SIZE = 8;  // Number of timesteps
    static constexpr int NN_FEATURE_SIZE = 12;  // Number of features per timestep (acc, omega, q, qd, p, v, tau, tau_sq_sum, v_cmd) for both legs
    
    // Constructor
    ContactNN(RobotModel& robot_ref);
    
    // Destructor
    ~ContactNN() = default;
    
    // Load ONNX contact estimation model
    void load_contact_model(const std::string& model_path);
    
    // Extract 23-element feature vector from current robot state
    Eigen::VectorXd extract_nn_features(
        const Eigen::VectorXd& encoder,
        const Eigen::VectorXd& encoderdot,
        const Eigen::Vector3d& imu_acc,
        const Eigen::Vector3d& imu_gyro,
        const Eigen::VectorXd& motor_torques,
        const Eigen::VectorXd& motor_torques_cmd,  // Uncomment if using commanded torques as features
        const Eigen::VectorXd& cmd_velocity         // Uncomment if using commanded velocity as a feature
    );
    
    // Add new feature vector to sliding window buffer
    void update_nn_feature_buffer(const Eigen::VectorXd& features);
    
    // Create [1, NN_WINDOW_SIZE, NN_FEATURE_SIZE] flattened tensor from buffer
    std::vector<float> create_nn_input_tensor();
    
    // // Normalize input: (data - mean) / std per feature across time dimension
    // void normalize_nn_input(std::vector<float>& input_features);
    
    // Run contact prediction inference (returns contacts; velocity output is not used)
    std::pair<Eigen::VectorXd, Eigen::VectorXd> predict_contact(const std::vector<float>& input_features);
    
    // Convenience method: create tensor, normalize, and predict (returns contacts; velocity output is not used)
    std::pair<Eigen::VectorXd, Eigen::VectorXd> nn_predict_contact();
    
    // Check if model is loaded and enabled
    bool is_enabled() const { return contact_nn_enabled; }
    
    // Clear the feature buffer (e.g., on reset)
    void clear_buffer() { nn_feature_window.clear(); }

private:
    // Reference to the robot model for FK/Jacobian access
    RobotModel& robot;
    
    // ONNX Runtime session and environment
    std::unique_ptr<Ort::Session> contact_model_session;
    std::unique_ptr<Ort::Env> ort_env;
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    
    // Model metadata
    std::vector<const char*> input_names;
    std::vector<const char*> output_names;
    std::vector<int64_t> input_shape;
    bool contact_nn_enabled = false;
    
    // Storage for dynamically allocated names
    std::string input_name_storage;
    std::vector<std::string> output_name_storage;
    
    // Sliding window buffer for feature history
    std::deque<Eigen::VectorXd> nn_feature_window;
};

#endif // CONTACT_NN_HPP
