#include <iostream>
#include "src/Z1_estimation/utility/RobotModel.hpp"

int main() {
    std::string config_path = "./config/digit_config.yaml";
    
    std::cout << "========================================" << std::endl;
    std::cout << "Testing RobotModel YAML Configuration" << std::endl;
    std::cout << "========================================" << std::endl;
    
    try {
        RobotModel robot(config_path);
        
        std::cout << "\n=== ROBOT IDENTIFICATION ===" << std::endl;
        std::cout << "Robot Name: " << robot.robot_name << std::endl;
        
        std::cout << "\n=== ROBOT STRUCTURE ===" << std::endl;
        std::cout << "Number of Legs: " << robot.num_legs << std::endl;
        std::cout << "Number of Contacts: " << robot.num_contacts << std::endl;
        std::cout << "Encoder Size (nq): " << robot.encoder_size << std::endl;
        std::cout << "Encoder Dot Size (nv): " << robot.encoderdot_size << std::endl;
        
        std::cout << "\n=== MODEL INFORMATION ===" << std::endl;
        std::cout << "Model Path: " << robot.robot_model_path << std::endl;
        std::cout << "Model Format: " << robot.model_format << std::endl;
        
        std::cout << "\n=== CONTACT FRAME IDs ===" << std::endl;
        std::cout << "Contact Point Frame IDs: [";
        for (size_t i = 0; i < robot.contact_point_frame_ids.size(); i++) {
            std::cout << robot.contact_point_frame_ids[i];
            if (i < robot.contact_point_frame_ids.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
        
        std::cout << "Foot Frame IDs: [";
        for (size_t i = 0; i < robot.foot_frame_ids.size(); i++) {
            std::cout << robot.foot_frame_ids[i];
            if (i < robot.foot_frame_ids.size() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
        
        std::cout << "\n=== FRAME TRANSFORMATIONS ===" << std::endl;
        std::cout << "IMU to Body Offset: [" << robot.IMU2BD.transpose() << "]" << std::endl;
        std::cout << "Gravity: [" << robot.Gravity.transpose() << "]" << std::endl;
        
        std::cout << "\n=== COVARIANCE CONSTANTS ===" << std::endl;
        std::cout << "Gyroscope: " << robot.cov_gyro_const << std::endl;
        std::cout << "Accelerometer: " << robot.cov_acc_const << std::endl;
        std::cout << "Encoder: " << robot.cov_enc_const << std::endl;
        std::cout << "Contact: " << robot.cov_contact_const << std::endl;
        std::cout << "Slip: " << robot.cov_slip_const << std::endl;
        std::cout << "Bias Gyro: " << robot.cov_bias_gyro_const << std::endl;
        std::cout << "Bias Accelerometer: " << robot.cov_bias_acc_const << std::endl;
        
        std::cout << "\n=== PRIOR COVARIANCES ===" << std::endl;
        std::cout << "Prior Orientation: " << robot.cov_prior_orientation_const << std::endl;
        std::cout << "Prior Velocity: " << robot.cov_prior_velocity_const << std::endl;
        std::cout << "Prior Position: " << robot.cov_prior_position_const << std::endl;
        std::cout << "Prior Bias Gyro: " << robot.cov_prior_bias_gyro_const << std::endl;
        std::cout << "Prior Bias Acc: " << robot.cov_prior_bias_acc_const << std::endl;
        
        std::cout << "\n=== CONTACT PARAMETERS ===" << std::endl;
        std::cout << "Variable Contact Cov Mode: " << (robot.variable_contact_cov_mode ? "true" : "false") << std::endl;
        std::cout << "Cov Amplifier: " << robot.cov_amplifier << std::endl;
        std::cout << "Slip Rejection Mode: " << (robot.slip_rejection_mode ? "true" : "false") << std::endl;
        std::cout << "Slip Threshold: " << robot.slip_threshold << std::endl;
        
        std::cout << "\n=== PINOCCHIO MODEL INFO ===" << std::endl;
        std::cout << "Number of Joints: " << robot.pin_model.njoints << std::endl;
        std::cout << "Number of Frames: " << robot.pin_model.nframes << std::endl;
        std::cout << "Model Name: " << robot.pin_model.name << std::endl;
        
        std::cout << "\n========================================" << std::endl;
        std::cout << "✓ All parameters loaded successfully!" << std::endl;
        std::cout << "========================================" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "\n✗ ERROR: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
