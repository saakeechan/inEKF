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
        std::cout << "Input shape: [" << input_shape[0] << ", " << input_shape[1] << ", " << input_shape[2] << "]" << std::endl;
        
    } catch (const Ort::Exception& e) {
        std::cerr << "Error loading ONNX contact model: " << e.what() << std::endl;
        contact_nn_enabled = false;
    }
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> ContactNN::predict_contact(const std::vector<float>& input_features) {
    if (!contact_nn_enabled) {
        std::cerr << "Warning: Contact model not loaded. Using default contact detection." << std::endl;
        return {Eigen::VectorXd::Zero(robot.num_legs), Eigen::VectorXd::Zero(robot.num_legs)};
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
        // - contact_logits: [batch_size, 1] = [1, 1] with logit for left leg (current timestep)
        // - foot_velocities: [batch_size, 1] = [1, 1] with velocity norm for left leg (current timestep)
        auto output_tensors = contact_model_session->Run(
            Ort::RunOptions{nullptr},
            input_names.data(),
            &input_tensor,
            1,
            output_names.data(),
            output_names.size()
        );
        
        // Get contact logits from first output
        float* contact_logits_data = output_tensors[2].GetTensorMutableData<float>();
        
        // Output shape is [batch_size, 1] = [1, 1] (left leg only, current timestep)
        // Apply sigmoid to convert logits to probabilities [0, 1]
        Eigen::VectorXd contact_predictions = Eigen::VectorXd::Zero(robot.num_legs);
        for (int leg = 0; leg < robot.num_legs; ++leg) {
            float logit = contact_logits_data[leg];  // Get logit for this leg
            float prob = 1.0f / (1.0f + std::exp(-logit));  // Sigmoid function
            contact_predictions(leg) = prob;  // Store probability
            // std::cout << "Leg " << leg << " - Logit: " << logit << ", Prob: " << prob << std::endl;
        }
        
        // Get foot velocity norms from second output
        float* foot_velocity_data = output_tensors[1].GetTensorMutableData<float>();
        
        // Output shape is [batch_size, 1] = [1, 1] (left leg only, current timestep)
        Eigen::VectorXd foot_velocities = Eigen::VectorXd::Zero(robot.num_legs);
        for (int leg = 0; leg < robot.num_legs; ++leg) {
            foot_velocities(leg) = foot_velocity_data[leg];  // Current timestep for each leg
        }
        
        return {contact_predictions, foot_velocities};
        
    } catch (const Ort::Exception& e) {
        std::cerr << "Error during ONNX contact prediction: " << e.what() << std::endl;
        return {Eigen::VectorXd::Zero(robot.num_legs), Eigen::VectorXd::Zero(robot.num_legs)};
    }
}

Eigen::VectorXd ContactNN::extract_nn_features(
    const Eigen::VectorXd& encoder,
    const Eigen::VectorXd& encoderdot,
    const Eigen::Vector3d& imu_acc,
    const Eigen::Vector3d& imu_gyro,
    const Eigen::VectorXd& motor_torques,
    const Eigen::VectorXd& motor_pos_cmd,
    const Eigen::VectorXd& cmd_velocity
) {
    // Extract 56-element feature vector for both legs.
    // Order: acc, omega, q, qd, p, v, tau, tau_sq_sum
    // For G1 (both legs): 3 + 3 + 12 + 12 + 6 + 6 + 12 + 2 = 56 features
    
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
    for (int leg = 0; leg < robot.num_legs; ++leg) {
        for (int joint_idx : robot.Leg_indices_map[leg]) {
            features(idx++) = encoder(joint_idx);
        }
    }

    // features(idx++) = std::abs(encoder(3));
    // features(idx++) = std::abs(encoder(4));
    
    // features(idx++) = std::abs(encoder(9));
    // features(idx++) = std::abs(encoder(10));
    
    // qd: joint velocities (left leg first, right leg second)
    for (int leg = 0; leg < robot.num_legs; ++leg) {
        for (int joint_idx : robot.Leg_indices_map[leg]) {
            features(idx++) = encoderdot(joint_idx);
        }
    }

    // features(idx++) = encoderdot(0);
    // features(idx++) = encoderdot(3);
    // features(idx++) = encoderdot(3);
    
    // p: foot positions (left foot xyz, right foot xyz)
    for (int leg = 0; leg < robot.num_legs; ++leg) {
        Eigen::Vector3d foot_pos = robot.get_foot_position(leg + 1);
        features(idx++) = foot_pos(0);
        features(idx++) = foot_pos(1);
        features(idx++) = foot_pos(2);
    }
    
    // v: foot velocities (left foot xyz, right foot xyz)
    for (int leg = 0; leg < robot.num_legs; ++leg) {
        Eigen::Vector3d foot_vel = robot.get_foot_velocity(leg + 1);
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

    // features(idx++) = std::abs(motor_torques(1));
    // features(idx++) = std::abs(motor_torques(2));
    // features(idx++) = std::abs(motor_torques(3));
    // features(idx++) = std::abs(motor_torques(4));
    // features(idx++) = std::abs(motor_torques(5));

    // features(idx++) = std::abs(motor_torques(7));
    // features(idx++) = std::abs(motor_torques(8));
    // features(idx++) = std::abs(motor_torques(9));
    // features(idx++) = std::abs(motor_torques(10));
    // features(idx++) = std::abs(motor_torques(11));

    // tau_mse: sum of squared torques for each leg
    for (int leg = 0; leg < robot.num_legs; ++leg) {
        double sum = 0.0;
        for (int joint_idx : robot.Leg_indices_map[leg]) {
            sum += motor_torques(joint_idx) * motor_torques(joint_idx);
        }
        features(idx++) = sum;
    }
    
    // // tau_cmd: commanded joint torques (left leg first, right leg second)
    // for (int leg = 0; leg < robot.num_legs; ++leg) {
    //     for (int joint_idx : robot.Leg_indices_map[leg]) {
    //         features(idx++) = motor_pos_cmd(joint_idx);
    //     }
    // }
    
    // // v_cmd: commanded velocity (1 element)
    // features(idx++) = cmd_velocity(0);
    
    // Validate that we filled exactly NN_FEATURE_SIZE elements
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
    // Total size: 1 * 10 * 57 = 570 elements
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

std::pair<Eigen::VectorXd, Eigen::VectorXd> ContactNN::nn_predict_contact() {
    // Create input tensor from buffer (returns flattened float vector)
    auto input_features = create_nn_input_tensor();
    
    // Run prediction (returns both contacts and velocities)
    return predict_contact(input_features);
}
