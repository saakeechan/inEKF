/* ----------------------------------------------------------------------------
 * Contact Neural Network Estimator Implementation
 * Handles neural network-based contact detection functionality
 * Note: NN outputs contact predictions only; foot velocities are computed via kinematics
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
        
        // Auto-detect input name
        auto input_name_ptr = contact_model_session->GetInputNameAllocated(0, allocator);
        input_name_storage = input_name_ptr.get();
        input_shape = {1, NN_WINDOW_SIZE, NN_FEATURE_SIZE};
        
        // Auto-detect output names (expecting 2 outputs: contact_logits and foot_velocities)
        size_t num_outputs = contact_model_session->GetOutputCount();
        std::cout << "Model has " << num_outputs << " outputs" << std::endl;
        
        // First, collect all output names into storage
        output_name_storage.reserve(num_outputs);
        for (size_t i = 0; i < num_outputs; i++) {
            auto output_name_ptr = contact_model_session->GetOutputNameAllocated(i, allocator);
            output_name_storage.push_back(output_name_ptr.get());
            std::cout << "Output " << i << ": " << output_name_storage[i] << std::endl;
        }
        
        // Now set up const char* pointers after all strings are stored
        input_names.clear();
        input_names.push_back(input_name_storage.c_str());
        
        output_names.clear();
        for (size_t i = 0; i < num_outputs; i++) {
            output_names.push_back(output_name_storage[i].c_str());
        }
        
        contact_nn_enabled = true;
        std::cout << "Contact model loaded successfully!" << std::endl;
        std::cout << "Input name: " << input_name_storage << std::endl;
        std::cout << "Input shape: [" << input_shape[0] << ", " << input_shape[1] << ", " << input_shape[2] << "]" << std::endl;
        
    } catch (const Ort::Exception& e) {
        std::cerr << "Error loading ONNX contact model: " << e.what() << std::endl;
        contact_nn_enabled = false;
    }
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> ContactNN::predict_contact(const std::vector<float>& input_features) {
    if (!contact_nn_enabled) {
        std::cerr << "Warning: Contact model not loaded. Using default contact detection." << std::endl;
        return {Eigen::VectorXd::Zero(robot.num_legs), Eigen::VectorXd::Zero(3 * robot.num_legs)};
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
        // Model outputs:
        // - contact_logits: [batch_size] = [1] with single logit for left leg only
        // - foot_velocities: [batch_size] = [1] with velocity norm for left leg (not used - computed via kinematics instead)
        auto output_tensors = contact_model_session->Run(
            Ort::RunOptions{nullptr},
            input_names.data(),
            &input_tensor,
            1,
            output_names.data(),
            output_names.size()
        );
        
        // Get contact logits from first output
        float* contact_logits_data = output_tensors[0].GetTensorMutableData<float>();
        auto output_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();
        
        // Output shape is [batch_size] = [1] for single scalar output (left leg only)
        // Apply sigmoid to convert logit to probability [0, 1]
        Eigen::VectorXd contact_predictions = Eigen::VectorXd::Zero(robot.num_legs);
        float logit_left = contact_logits_data[0];  // Left leg contact logit
        float prob_left = 1.0f / (1.0f + std::exp(-logit_left));  // Sigmoid function

        contact_predictions(0) = prob_left;  // Store probability for left leg
        // Right leg and any other legs remain zero
        
        // Extract foot velocity norm from second output
        // CNN outputs dv_norm directly as a single scalar for left leg
        Eigen::VectorXd foot_velocities = Eigen::VectorXd::Zero(3 * robot.num_legs);
        if (output_tensors.size() > 1) {
            float* velocity_data = output_tensors[1].GetTensorMutableData<float>();
            
            // Output shape is [batch_size] = [1] for single scalar output
            // Store dv_norm directly in first component
            foot_velocities(0) = velocity_data[0];  // Left leg dv_norm
            // Right leg and other components remain zero
        }
        
        return {contact_predictions, foot_velocities};
        
    } catch (const Ort::Exception& e) {
        std::cerr << "Error during ONNX contact prediction: " << e.what() << std::endl;
        return {Eigen::VectorXd::Zero(robot.num_legs), Eigen::VectorXd::Zero(3 * robot.num_legs)};
    }
}

Eigen::VectorXd ContactNN::extract_nn_features(
    const Eigen::VectorXd& encoder,
    const Eigen::VectorXd& encoderdot,
    const Eigen::Vector3d& imu_acc,
    const Eigen::Vector3d& imu_gyro,
    const Eigen::VectorXd& motor_torques,
    const Eigen::VectorXd& motor_torques_cmd,
    const Eigen::VectorXd& cmd_velocity
) {
    // Extract 31-element feature vector
    // Order: q, qd, acc, omega, p, v, tau, tau_sq_sum
    // For G1: 12 + 12 + 3 + 3 + 6 + 6 + 12 + 1 = 31 features
    
    Eigen::VectorXd features(NN_FEATURE_SIZE);
    int idx = 0;
    
    
    // acc: IMU acceleration (3 elements)
    features(idx++) = imu_acc(0);
    features(idx++) = imu_acc(1);
    features(idx++) = imu_acc(2);
    
    // omega: IMU angular velocity (3 elements)
    features(idx++) = imu_gyro(0);
    features(idx++) = imu_gyro(1);
    features(idx++) = imu_gyro(2);

    // q: joint positions (left leg first, right leg second)
    for (int leg = 0; leg < 1; ++leg) {
        for (int joint_idx : robot.Leg_indices_map[leg]) {
            features(idx++) = encoder(joint_idx);
        }
    }
    
    // qd: joint velocities (left leg first, right leg second)
    for (int leg = 0; leg < 1; ++leg) {
        for (int joint_idx : robot.Leg_indices_map[leg]) {
            features(idx++) = encoderdot(joint_idx);
        }
    }
    
    // p: foot positions (left foot xyz, right foot xyz)
    for (int leg = 1; leg <= 1; ++leg) {
        Eigen::Vector3d foot_pos = robot.get_foot_position(leg);
        features(idx++) = foot_pos(0);
        features(idx++) = foot_pos(1);
        features(idx++) = foot_pos(2);
    }
    
    // v: foot velocities (left foot xyz, right foot xyz)
    for (int leg = 1; leg <= 1; ++leg) {
        Eigen::Vector3d foot_vel = robot.get_foot_velocity(leg);
        features(idx++) = foot_vel(0);
        features(idx++) = foot_vel(1);
        features(idx++) = foot_vel(2);
    }
    
    // tau: joint torques (left leg first, right leg second)
    for (int leg = 0; leg < 1; ++leg) {
        for (int joint_idx : robot.Leg_indices_map[leg]) {
            features(idx++) = motor_torques(joint_idx);
        }
    }

    // qd: joint velocities (left leg first, right leg second)
    for (int leg = 0; leg < 1; ++leg) {
        int sum = 0;
        for (int joint_idx : robot.Leg_indices_map[leg]) {
            sum += motor_torques(joint_idx) * motor_torques(joint_idx);
        }
        features(idx++) = sum;  // Avoid division by zero
    }
    
    // // tau_cmd: commanded joint torques (left leg first, right leg second)
    // for (int leg = 0; leg < 1; ++leg) {
    //     for (int joint_idx : robot.Leg_indices_map[leg]) {
    //         features(idx++) = motor_torques_cmd(joint_idx);
    //     }
    // }
    
    // v_cmd: commanded velocity (1 element)
    features(idx++) = cmd_velocity(0);
    
    // Validate that we filled exactly 31 elements
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
    // Total size: 1 * 10 * 67 = 670 elements
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

// void ContactNN::normalize_nn_input(std::vector<float>& input_features) {
//     // Normalize input features: (data - mean) / std per feature across time dimension
//     // input_features shape: [1, NN_WINDOW_SIZE, NN_FEATURE_SIZE] = [1, 10, 67]
//     // Flattened as: batch * window * features = 1 * 10 * 67 = 670 elements
//     // Layout: [t0_f0, t0_f1, ..., t0_f66, t1_f0, t1_f1, ..., t1_f66, ..., t9_f66]
    
//     // For each feature dimension
//     for (int f = 0; f < NN_FEATURE_SIZE; ++f) {
//         // Compute mean for this feature across all time steps
//         float mean = 0.0f;
//         for (int t = 0; t < NN_WINDOW_SIZE; ++t) {
//             int idx = t * NN_FEATURE_SIZE + f;
//             mean += input_features[idx];
//         }
//         mean /= NN_WINDOW_SIZE;
        
//         // Compute std for this feature across all time steps
//         float variance = 0.0f;
//         for (int t = 0; t < NN_WINDOW_SIZE; ++t) {
//             int idx = t * NN_FEATURE_SIZE + f;
//             float diff = input_features[idx] - mean;
//             variance += diff * diff;
//         }
//         float std = std::sqrt(variance / NN_WINDOW_SIZE);
        
//         // Replace 0 std with 1 to avoid division by zero
//         if (std < 1e-8f) {
//             std = 1.0f;
//         }
        
//         // Normalize: (value - mean) / std
//         for (int t = 0; t < NN_WINDOW_SIZE; ++t) {
//             int idx = t * NN_FEATURE_SIZE + f;
//             input_features[idx] = (input_features[idx] - mean) / std;
//         }
//     }
// }

std::pair<Eigen::VectorXd, Eigen::VectorXd> ContactNN::nn_predict_contact() {
    // Create input tensor from buffer (returns flattened float vector)
    auto input_features = create_nn_input_tensor();
    
    // Run prediction (returns both contacts and velocities)
    return predict_contact(input_features);
}
