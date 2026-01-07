//
// Created by Saaketh 01/05/26
// Digit-Specific Robot Model Implementation with Pinocchio
//

#include "Robot_Model_Digit.hpp"
#include "DigitCSVData.hpp"
#include <stdio.h>
#include <iostream>
#include <pinocchio/parsers/mjcf.hpp>


Robot_Model_Digit::Robot_Model_Digit() {
    Gravity << 0.0, 0.0, -9.80665;
    IMU2BD.setZero();
    
    // Load robot model
    if (use_pinocchio) {
        load_xml(robot_model_path, contact_point_frame_ids);
    }
}

Robot_Model_Digit::~Robot_Model_Digit() {

}

void Robot_Model_Digit::load_xml(const std::string& xml_path, const std::vector<int>& contact_frame_ids) {
    try {
        pinocchio::mjcf::buildModel(xml_path, pin_model);
        pin_data = pinocchio::Data(pin_model);
        
        std::cout << "Successfully loaded robot XML from: " << xml_path << std::endl;
        std::cout << "Number of joints: " << pin_model.njoints << std::endl;
        std::cout << "Number of frames: " << pin_model.nframes << std::endl;
        
        // Setup contact frame IDs
        foot_frame_ids.clear();
        if (!contact_frame_ids.empty()) {
            foot_frame_ids.resize(contact_frame_ids.size());
            for (size_t i = 0; i < contact_frame_ids.size(); i++) {
                foot_frame_ids[i] = contact_frame_ids[i];
            }
            std::cout << "Loaded " << foot_frame_ids.size() << " contact frames" << std::endl;
        } else {
            std::cerr << "Warning: No contact frame IDs provided" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Failed to load XML: " << e.what() << std::endl;
        std::cerr << "Disabling Pinocchio for this robot model" << std::endl;
        std::cerr << "XML path: " << xml_path << std::endl;
        use_pinocchio = false;
        std::exit(1);  // Terminates the entire program with exit code 1
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////     Main Kinematics Function  ////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

// Main function: Computes forward kinematics and Jacobians for all frames
// This is the most efficient way - computes FK once and then all Jacobians can be computed
void Robot_Model_Digit::Forward_Kinematics(const Eigen::VectorXd &q_leg, const Eigen::VectorXd &q_dot_leg) const {
    
    if (!use_pinocchio || foot_frame_ids.empty()) {
        std::cerr << "Error: Pinocchio not initialized properly" << std::endl;
        return;
    }
    
    // Compute forward kinematics - this updates all joint and frame placements
    pinocchio::forwardKinematics(pin_model, const_cast<pinocchio::Data&>(pin_data), q_leg, q_dot_leg);
    
    // Update all frame placements (foot frames, etc.)
    pinocchio::updateFramePlacements(pin_model, const_cast<pinocchio::Data&>(pin_data));
    
    // Note: After this call, the following data is available:
    // - pin_data.oMf[frame_id] contains the SE3 transformation for each frame
    // - Frame positions can be accessed via pin_data.oMf[frame_id].translation()
    // - Jacobians can be computed using computeFrameJacobian() which will be efficient
    //   since the kinematics are already computed
}

// Retrieve foot velocity after calling Forward_Kinematics
Eigen::Vector3d Robot_Model_Digit::get_foot_velocity(int i_leg) const {
    
    if (!use_pinocchio || foot_frame_ids.empty()) {
        std::cerr << "Error: Pinocchio not initialized properly" << std::endl;
        return Eigen::Vector3d::Zero();
    }
    
    if (i_leg < 1 || i_leg > static_cast<int>(foot_frame_ids.size())) {
        std::cerr << "Error: Invalid leg index " << i_leg << ". Must be between 1 and " << foot_frame_ids.size() << std::endl;
        return Eigen::Vector3d::Zero();
    }
    
    // Compute frame velocity in World frame
    pinocchio::Motion foot_velocity = pinocchio::getFrameVelocity(
        pin_model,
        const_cast<pinocchio::Data&>(pin_data),
        foot_frame_ids[i_leg - 1],
        pinocchio::WORLD
    );
    
    // Extract linear velocity component
    return foot_velocity.linear();
}

// Retrieve foot position after calling Forward_Kinematics
Eigen::Vector3d Robot_Model_Digit::get_foot_position(int i_leg) const {
    
    if (!use_pinocchio || foot_frame_ids.empty()) {
        std::cerr << "Error: Pinocchio not initialized properly" << std::endl;
        return Eigen::Vector3d::Zero();
    }
    
    if (i_leg < 1 || i_leg > static_cast<int>(foot_frame_ids.size())) {
        std::cerr << "Error: Invalid leg index " << i_leg << ". Must be between 1 and " << foot_frame_ids.size() << std::endl;
        return Eigen::Vector3d::Zero();
    }
    
    // Get foot position from the already-computed forward kinematics
    const pinocchio::SE3& foot_placement = pin_data.oMf[foot_frame_ids[i_leg - 1]];
    Eigen::Vector3d foot_position = foot_placement.translation();
    
    return foot_position + IMU2BD;
}

// Retrieve foot Jacobian after calling Forward_Kinematics
Eigen::MatrixXd Robot_Model_Digit::get_foot_jacobian(int i_leg) const {
    
    if (!use_pinocchio || foot_frame_ids.empty()) {
        std::cerr << "Error: Pinocchio not initialized properly" << std::endl;
        return Eigen::MatrixXd::Zero(3, pin_model.nv);
    }
    
    if (i_leg < 1 || i_leg > static_cast<int>(foot_frame_ids.size())) {
        std::cerr << "Error: Invalid leg index " << i_leg << ". Must be between 1 and " << foot_frame_ids.size() << std::endl;
        return Eigen::MatrixXd::Zero(3, pin_model.nv);
    }
    
    // Compute frame Jacobian in World frame
    // This is efficient because Forward_Kinematics() already computed the kinematics
    Eigen::MatrixXd J_full = Eigen::MatrixXd::Zero(6, pin_model.nv);
    pinocchio::getFrameJacobian(
        pin_model,
        const_cast<pinocchio::Data&>(pin_data),
        foot_frame_ids[i_leg - 1],
        pinocchio::WORLD,
        J_full
    );
    
    // Extract linear velocity part (first 3 rows)
    return J_full.topRows(3);
}

// Compute joint Jacobians for efficient Jacobian queries
// Call this after Forward_Kinematics and before get_foot_jacobian for better performance
// Uses the configuration already stored in pin_data from Forward_Kinematics
void Robot_Model_Digit::compute_joint_jacobians() const {
    
    if (!use_pinocchio) {
        std::cerr << "Error: Pinocchio not initialized properly" << std::endl;
        return;
    }
    
    // Compute all joint Jacobians using the configuration already in pin_data
    // This prepares the data structure for efficient frame Jacobian queries
    pinocchio::computeJointJacobians(pin_model, const_cast<pinocchio::Data&>(pin_data));
}


void Robot_Model_Digit::Covariance_Reset(Eigen::Matrix<double, 12, 1> cov_val_setting) {

    cov_gyro_const = cov_val_setting(0);
    cov_acc_const = cov_val_setting(1);
    cov_slip_const = cov_val_setting(2);
    cov_contact_const = cov_val_setting(3);
    cov_enc_const = cov_val_setting(4);
    cov_bias_gyro_const = cov_val_setting(5);
    cov_bias_acc_const = cov_val_setting(6);
    cov_prior_orientation_const = cov_val_setting(7);
    cov_prior_velocity_const = cov_val_setting(8);
    cov_prior_position_const = cov_val_setting(9);
    cov_prior_bias_gyro_const = cov_val_setting(10);
    cov_prior_bias_acc_const = cov_val_setting(11);

    Covariance_Gyro = cov_gyro_const * Eigen::Matrix<double, 3, 3>::Identity();
    Covariance_Acc = cov_acc_const * Eigen::Matrix<double, 3, 3>::Identity();
    Covariance_Slip = cov_slip_const * Eigen::Matrix<double, 3, 3>::Identity();
    Covariance_Contact = cov_contact_const * Eigen::Matrix<double, 3, 3>::Identity();
    Covariance_Encoder = cov_enc_const * Eigen::Matrix<double, 3, 3>::Identity();

    Covariance_Bias_Gyro = cov_bias_gyro_const * Eigen::Matrix<double, 3, 3>::Identity();
    Covariance_Bias_Acc = cov_bias_acc_const * Eigen::Matrix<double, 3, 3>::Identity();

    Covariance_Prior_Orientation = cov_prior_orientation_const * Eigen::Matrix<double, 3, 3>::Identity();
    Covariance_Prior_Velocity = cov_prior_velocity_const * Eigen::Matrix<double, 3, 3>::Identity();
    Covariance_Prior_Position = cov_prior_position_const * Eigen::Matrix<double, 3, 3>::Identity();

    Covariance_Prior_Bias_Gyro = cov_prior_bias_gyro_const * Eigen::Matrix<double, 3, 3>::Identity();
    Covariance_Prior_Bias_Acc = cov_prior_bias_acc_const * Eigen::Matrix<double, 3, 3>::Identity();

    SQRT_INFO_Covariance_Gyro = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_gyro_const);
    SQRT_INFO_Covariance_Acc = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_acc_const);
    SQRT_INFO_Covariance_Slip = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_slip_const);
    SQRT_INFO_Covariance_Contact = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_contact_const);
    SQRT_INFO_Covariance_Encoder = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_enc_const);
    SQRT_INFO_Covariance_Bias_Gyro = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_bias_gyro_const);
    SQRT_INFO_Covariance_Bias_Acc = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_bias_acc_const);
    SQRT_INFO_Covariance_Prior_Orientation =
    Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_prior_orientation_const);
    SQRT_INFO_Covariance_Prior_Velocity = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_prior_velocity_const);
    SQRT_INFO_Covariance_Prior_Position = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_prior_position_const);
    SQRT_INFO_Covariance_Prior_Bias_Gyro = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_prior_bias_gyro_const);
    SQRT_INFO_Covariance_Prior_Bias_Acc = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_prior_bias_acc_const);
}

// ========================================================================
// DIGIT-SPECIFIC ENCODER ASSIGNMENT FROM CSV DATA
// Maps CSV columns to Pinocchio configuration indices for Digit robot
// ========================================================================

void Robot_Model_Digit::assign_encoder_from_csv(Eigen::Vector<double, ENCODER_SIZE>& encoder, 
                                                 const DigitCSVData& csvData, 
                                                 int j) const {
    // Populate ENCODER with 61 configuration values for Pinocchio FK
    // q[0:7] → baseJoint_0 (position + quaternion) - set to identity (0,0,0,0,0,0,1)
    encoder(0) = 0.0;
    encoder(1) = 0.0;
    encoder(2) = 0.0;
    encoder(3) = 0.0;
    encoder(4) = 0.0;
    encoder(5) = 0.0;
    encoder(6) = 1.0;
    
    // q[7:10] → Left hip (roll, yaw, pitch)
    encoder(7) = csvData.left_hip_roll[j];
    encoder(8) = csvData.left_hip_yaw[j];
    encoder(9) = csvData.left_hip_pitch[j];
    
    // q[10:14] → left-achilles-rod (spherical joint) - set to zeros
    encoder(10) = 0.0;
    encoder(11) = 0.0;
    encoder(12) = 0.0;
    encoder(13) = 0.0;
    
    // q[14:18] → Left knee, shin, tarsus, heel-spring
    encoder(14) = csvData.left_knee[j];
    encoder(15) = csvData.left_shin[j];
    encoder(16) = csvData.left_tarsus[j];
    encoder(17) = csvData.left_heel_spring[j];
    
    // q[18:19] → left-toe-A
    encoder(18) = csvData.left_toe_A[j];
    
    // q[19:23] → left-toe-A-rod (spherical joint) - set to zeros
    encoder(19) = 0.0;
    encoder(20) = 0.0;
    encoder(21) = 0.0;
    encoder(22) = 0.0;
    
    // q[23] → left-toe-B
    encoder(23) = csvData.left_toe_B[j];
    
    // q[24:28] → left-toe-B-rod (spherical joint) - set to zeros
    encoder(24) = 0.0;
    encoder(25) = 0.0;
    encoder(26) = 0.0;
    encoder(27) = 0.0;
    
    // q[28:30] → left toe pitch and roll
    encoder(28) = csvData.left_toe_pitch[j];
    encoder(29) = csvData.left_toe_roll[j];
    
    // q[30:34] → Left arm (shoulder roll, pitch, yaw, elbow) - set to zeros for now
    encoder(30) = 0.0; // csvData.left_shoulder_roll[j];
    encoder(31) = 0.0; // csvData.left_shoulder_pitch[j];
    encoder(32) = 0.0; // csvData.left_shoulder_yaw[j];
    encoder(33) = 0.0; // csvData.left_elbow[j];
    
    // q[34:37] → Right hip (roll, yaw, pitch)
    encoder(34) = csvData.right_hip_roll[j];
    encoder(35) = csvData.right_hip_yaw[j];
    encoder(36) = csvData.right_hip_pitch[j];
    
    // q[37:41] → right-achilles-rod (spherical joint) - set to zeros
    encoder(37) = 0.0;
    encoder(38) = 0.0;
    encoder(39) = 0.0;
    encoder(40) = 0.0;
    
    // q[41:45] → Right knee, shin, tarsus, heel-spring
    encoder(41) = csvData.right_knee[j];
    encoder(42) = csvData.right_shin[j];
    encoder(43) = csvData.right_tarsus[j];
    encoder(44) = csvData.right_heel_spring[j];
    
    // q[45] → right-toe-A
    encoder(45) = csvData.right_toe_A[j];
    
    // q[46:50] → right-toe-A-rod (spherical joint) - set to zeros
    encoder(46) = 0.0;
    encoder(47) = 0.0;
    encoder(48) = 0.0;
    encoder(49) = 0.0;
    
    // q[50] → right-toe-B
    encoder(50) = csvData.right_toe_B[j];
    
    // q[51:55] → right-toe-B-rod (spherical joint) - set to zeros
    encoder(51) = 0.0;
    encoder(52) = 0.0;
    encoder(53) = 0.0;
    encoder(54) = 0.0;
    
    // q[55:57] → right toe pitch and roll
    encoder(55) = csvData.right_toe_pitch[j];
    encoder(56) = csvData.right_toe_roll[j];
    
    // q[57:61] → Right arm (shoulder roll, pitch, yaw, elbow) - set to zeros for now
    encoder(57) = 0.0; // csvData.right_shoulder_roll[j];
    encoder(58) = 0.0; // csvData.right_shoulder_pitch[j];
    encoder(59) = 0.0; // csvData.right_shoulder_yaw[j];
    encoder(60) = 0.0; // csvData.right_elbow[j];
}

void Robot_Model_Digit::assign_encoderdot_from_csv(Eigen::Vector<double, ENCODERDOT_SIZE>& encoderdot, 
                                                     const DigitCSVData& csvData, 
                                                     int j) const {
    // ENCODERDOT - populate with velocity data (54 velocity DOFs)
    // Base velocities (linear) - set to zero (base should not move in FK)
    encoderdot(0) = 0.0;
    encoderdot(1) = 0.0;
    encoderdot(2) = 0.0;
    // Base angular velocities - set to zero (base should not move in FK)
    encoderdot(3) = 0.0;
    encoderdot(4) = 0.0;
    encoderdot(5) = 0.0;
    
    // Left leg joint velocities
    encoderdot(6) = csvData.left_hip_roll_vel[j];
    encoderdot(7) = csvData.left_hip_yaw_vel[j];
    encoderdot(8) = csvData.left_hip_pitch_vel[j];
    encoderdot(9) = 0.0; // achilles-rod
    encoderdot(10) = csvData.left_knee_vel[j];
    encoderdot(11) = csvData.left_shin_vel[j];
    encoderdot(12) = csvData.left_tarsus_vel[j];
    encoderdot(13) = csvData.left_heel_spring_vel[j];
    encoderdot(14) = csvData.left_toe_A_vel[j];
    encoderdot(15) = 0.0; // toe-A-rod
    encoderdot(16) = csvData.left_toe_B_vel[j];
    encoderdot(17) = 0.0; // toe-B-rod
    encoderdot(18) = csvData.left_toe_pitch_vel[j];
    encoderdot(19) = csvData.left_toe_roll_vel[j];
    
    // Left arm velocities - set to zeros
    encoderdot(20) = 0.0;
    encoderdot(21) = 0.0;
    encoderdot(22) = 0.0;
    encoderdot(23) = 0.0;
    
    // Right leg joint velocities
    encoderdot(24) = csvData.right_hip_roll_vel[j];
    encoderdot(25) = csvData.right_hip_yaw_vel[j];
    encoderdot(26) = csvData.right_hip_pitch_vel[j];
    encoderdot(27) = 0.0; // achilles-rod
    encoderdot(28) = csvData.right_knee_vel[j];
    encoderdot(29) = csvData.right_shin_vel[j];
    encoderdot(30) = csvData.right_tarsus_vel[j];
    encoderdot(31) = csvData.right_heel_spring_vel[j];
    encoderdot(32) = csvData.right_toe_A_vel[j];
    encoderdot(33) = 0.0; // toe-A-rod
    encoderdot(34) = csvData.right_toe_B_vel[j];
    encoderdot(35) = 0.0; // toe-B-rod
    encoderdot(36) = csvData.right_toe_pitch_vel[j];
    encoderdot(37) = csvData.right_toe_roll_vel[j];
    
    // Right arm velocities - set to zeros
    encoderdot(38) = 0.0;
    encoderdot(39) = 0.0;
    encoderdot(40) = 0.0;
    encoderdot(41) = 0.0;
}

