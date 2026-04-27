/* ----------------------------------------------------------------------------
 * Contact Neural Network Estimator Implementation
 * Handles all neural network-based contact detection functionality
 * -------------------------------------------------------------------------- */

#include "ContactNN.hpp"
#include <iostream>
#include <cmath>

ContactNN::ContactNN(RobotModel& robot_ref) : robot(robot_ref) {
    // Initialize empty buffers
    nn_feature_window.clear();
}

void ContactNN::load_contact_model(const std::string& model_path) {
    try {
        std::cout << "Loading ONNX contact model from: " << model_path << std::endl;
        
        // Create ONNX Runtime environment
        ort_env = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "ContactModel");
        
        // Configure session options
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(1);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
        
        // Create session
        contact_model_session = std::make_unique<Ort::Session>(*ort_env, model_path.c_str(), session_options);
        
        // Get input/output names and shape
        Ort::AllocatorWithDefaultOptions allocator;
        
        // Input: "input" with shape [batch, seq_len, features] = [1, 25, 54]
        input_names = {"input"};
        input_shape = {1, NN_WINDOW_SIZE, NN_FEATURE_SIZE};
        
        // Output: "output" with shape [batch, num_classes] = [1, 4]
        output_names = {"output"};
        
        contact_nn_enabled = true;
        std::cout << "Contact model loaded successfully!" << std::endl;
        std::cout << "Input shape: [" << input_shape[0] << ", " << input_shape[1] << ", " << input_shape[2] << "]" << std::endl;
        
    } catch (const Ort::Exception& e) {
        std::cerr << "Error loading ONNX contact model: " << e.what() << std::endl;
        contact_nn_enabled = false;
    }
}

std::vector<bool> ContactNN::predict_contact(const std::vector<float>& input_features) {
    if (!contact_nn_enabled) {
        std::cerr << "Warning: Contact model not loaded. Using default contact detection." << std::endl;
        return std::vector<bool>(robot.num_legs, false);
    }
    
    try {
        // Create input tensor from flattened feature vector
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info,
            const_cast<float*>(input_features.data()),
            input_features.size(),
            input_shape.data(),
            input_shape.size()
        );
        
        // Run inference
        // Output shape: [batch_size, 4] with logits for 4 contact classes
        // Class 0 = 00 (both legs in air)
        // Class 1 = 01 (leg 0 air, leg 1 contact)
        // Class 2 = 10 (leg 0 contact, leg 1 air)
        // Class 3 = 11 (both legs in contact)
        auto output_tensors = contact_model_session->Run(
            Ort::RunOptions{nullptr},
            input_names.data(),
            &input_tensor,
            1,
            output_names.data(),
            1
        );
        
        // Get output tensor data
        float* output_data = output_tensors[0].GetTensorMutableData<float>();
        auto output_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
        int num_classes = output_shape[1];  // Should be 4
        
        // Find argmax (predicted class)
        int predicted_class = 0;
        float max_logit = output_data[0];
        for (int i = 1; i < num_classes; i++) {
            if (output_data[i] > max_logit) {
                max_logit = output_data[i];
                predicted_class = i;
            }
        }
        
        // Decode decimal class to binary contact states
        // For 2 legs: extract bits from predicted_class
        std::vector<bool> contact_predictions(robot.num_legs);
        for (int leg = robot.num_legs - 1; leg >= 0; --leg) {
            contact_predictions[leg] = (predicted_class & (1 << (robot.num_legs - 1 - leg))) != 0;
        }
        
        return contact_predictions;
        
    } catch (const Ort::Exception& e) {
        std::cerr << "Error during ONNX contact prediction: " << e.what() << std::endl;
        return std::vector<bool>(robot.num_legs, false);
    }
}

Eigen::VectorXd ContactNN::extract_nn_features(
    const Eigen::VectorXd& encoder,
    const Eigen::VectorXd& encoderdot,
    const Eigen::Vector3d& imu_acc,
    const Eigen::Vector3d& imu_gyro,
    const Eigen::VectorXd& motor_torques
) {
    // Extract 54-element feature vector
    // Order: q, qd, acc, omega, p, v, tau
    // For G1: 12 + 12 + 3 + 3 + 6 + 6 + 12 = 54
    
    Eigen::VectorXd features(NN_FEATURE_SIZE);
    int idx = 0;
    
    // q: joint positions (left leg first, right leg second)
    for (int leg = 0; leg < robot.num_legs; ++leg) {
        for (int joint_idx : robot.Leg_indices_map[leg]) {
            features(idx++) = encoder(joint_idx);
        }
    }
    
    // qd: joint velocities (left leg first, right leg second)
    for (int leg = 0; leg < robot.num_legs; ++leg) {
        for (int joint_idx : robot.Leg_indices_map[leg]) {
            features(idx++) = encoderdot(joint_idx);
        }
    }
    
    // acc: IMU acceleration (3 elements)
    features(idx++) = imu_acc(0);
    features(idx++) = imu_acc(1);
    features(idx++) = imu_acc(2);
    
    // omega: IMU angular velocity (3 elements)
    features(idx++) = imu_gyro(0);
    features(idx++) = imu_gyro(1);
    features(idx++) = imu_gyro(2);
    
    // p: foot positions (left foot xyz, right foot xyz)
    for (int leg = 1; leg <= robot.num_legs; ++leg) {
        Eigen::Vector3d foot_pos = robot.get_foot_position(leg);
        features(idx++) = foot_pos(0);
        features(idx++) = foot_pos(1);
        features(idx++) = foot_pos(2);
    }
    
    // v: foot velocities (left foot xyz, right foot xyz)
    for (int leg = 1; leg <= robot.num_legs; ++leg) {
        Eigen::Vector3d foot_vel = robot.get_foot_velocity(leg);
        features(idx++) = foot_vel(0);
        features(idx++) = foot_vel(1);
        features(idx++) = foot_vel(2);
    }
    
    // tau: joint torques (left leg first, right leg second)
    for (int leg = 0; leg < robot.num_legs; ++leg) {
        for (int joint_idx : robot.Leg_indices_map[leg]) {
            features(idx++) = motor_torques(joint_idx);
        }
    }
    
    // Validate that we filled exactly 54 elements
    if (idx != NN_FEATURE_SIZE) {
        std::cerr << "Warning: Extracted " << idx << " features, expected " 
                  << NN_FEATURE_SIZE << std::endl;
    }
    
    return features;
}

void ContactNN::update_nn_feature_buffer(const Eigen::VectorXd& features) {
    // Add new feature vector to the end of the buffer
    nn_feature_window.push_back(features);
    
    // If buffer exceeds window size, remove oldest entry
    if (nn_feature_window.size() > NN_WINDOW_SIZE) {
        nn_feature_window.pop_front();
    }
}

std::vector<float> ContactNN::create_nn_input_tensor() {
    // Create flattened float vector [1 * NN_WINDOW_SIZE * NN_FEATURE_SIZE]
    // Total size: 1 * 25 * 54 = 1350 elements
    int total_size = NN_WINDOW_SIZE * NN_FEATURE_SIZE;
    std::vector<float> input_tensor(total_size, 0.0f);
    
    int buffer_size = nn_feature_window.size();
    int start_idx = NN_WINDOW_SIZE - buffer_size;
    
    // Vectorized copy using Eigen for SIMD acceleration
    for (int t = 0; t < buffer_size; ++t) {
        // Map output location as Eigen array for vectorized type conversion
        Eigen::Map<Eigen::ArrayXf> dst_map(
            input_tensor.data() + (start_idx + t) * NN_FEATURE_SIZE, 
            NN_FEATURE_SIZE
        );
        // Convert double to float in one vectorized SIMD operation
        dst_map = nn_feature_window[t].cast<float>();
    }
    
    return input_tensor;
}

void ContactNN::normalize_nn_input(std::vector<float>& input_features) {
    // Normalize input features: (data - mean) / std per feature across time dimension
    // input_features shape: [1, NN_WINDOW_SIZE, NN_FEATURE_SIZE] = [1, 25, 54]
    // Flattened as: batch * window * features = 1 * 25 * 54 = 1350 elements
    // Layout: [t0_f0, t0_f1, ..., t0_f53, t1_f0, t1_f1, ..., t1_f53, ..., t24_f53]
    
    // For each feature dimension
    for (int f = 0; f < NN_FEATURE_SIZE; ++f) {
        // Compute mean for this feature across all time steps
        float mean = 0.0f;
        for (int t = 0; t < NN_WINDOW_SIZE; ++t) {
            int idx = t * NN_FEATURE_SIZE + f;
            mean += input_features[idx];
        }
        mean /= NN_WINDOW_SIZE;
        
        // Compute std for this feature across all time steps
        float variance = 0.0f;
        for (int t = 0; t < NN_WINDOW_SIZE; ++t) {
            int idx = t * NN_FEATURE_SIZE + f;
            float diff = input_features[idx] - mean;
            variance += diff * diff;
        }
        float std = std::sqrt(variance / NN_WINDOW_SIZE);
        
        // Replace 0 std with 1 to avoid division by zero
        if (std < 1e-8f) {
            std = 1.0f;
        }
        
        // Normalize: (value - mean) / std
        for (int t = 0; t < NN_WINDOW_SIZE; ++t) {
            int idx = t * NN_FEATURE_SIZE + f;
            input_features[idx] = (input_features[idx] - mean) / std;
        }
    }
}

std::vector<bool> ContactNN::nn_predict_contact() {
    // Create input tensor from buffer (returns flattened float vector)
    auto input_features = create_nn_input_tensor();
    
    // Normalize features
    normalize_nn_input(input_features);
    
    // Run prediction
    return predict_contact(input_features);
}
