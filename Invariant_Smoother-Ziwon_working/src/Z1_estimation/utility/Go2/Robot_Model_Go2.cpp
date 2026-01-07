//
// Created by Saaketh 01/05/26
// Go2-Specific Robot Model Implementation with Pinocchio
//

#include "Robot_Model_Go2.hpp"
#include "Go2CSVData.hpp"
#include <stdio.h>
#include <iostream>
#include <pinocchio/parsers/mjcf.hpp>


Robot_Model_Go2::Robot_Model_Go2() {
    Gravity << 0.0, 0.0, -9.80665;
    IMU2BD.setZero();
    
    // Load robot model
    if (use_pinocchio) {
        load_xml(robot_model_path, contact_point_frame_ids);
    }
}

Robot_Model_Go2::~Robot_Model_Go2() {

}

void Robot_Model_Go2::load_xml(const std::string& xml_path, const std::vector<int>& contact_frame_ids) {
    try {

        if (xml_path.substr(xml_path.find_last_of(".") + 1) == "urdf") {
            pinocchio::urdf::buildModel(xml_path, pin_model);
        } else {
            pinocchio::mjcf::buildModel(xml_path, pin_model);
        }
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
void Robot_Model_Go2::Forward_Kinematics(const Eigen::VectorXd &q_leg, const Eigen::VectorXd &q_dot_leg) const {
    
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
Eigen::Vector3d Robot_Model_Go2::get_foot_velocity(int i_leg) const {
    
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
Eigen::Vector3d Robot_Model_Go2::get_foot_position(int i_leg) const {
    
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
Eigen::MatrixXd Robot_Model_Go2::get_foot_jacobian(int i_leg) const {
    
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
void Robot_Model_Go2::compute_joint_jacobians() const {
    
    if (!use_pinocchio) {
        std::cerr << "Error: Pinocchio not initialized properly" << std::endl;
        return;
    }
    
    // Compute all joint Jacobians using the configuration already in pin_data
    // This prepares the data structure for efficient frame Jacobian queries
    pinocchio::computeJointJacobians(pin_model, const_cast<pinocchio::Data&>(pin_data));
}


void Robot_Model_Go2::Covariance_Reset(Eigen::Matrix<double, 12, 1> cov_val_setting) {

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
// Go2-SPECIFIC ENCODER ASSIGNMENT FROM CSV DATA
// Maps CSV columns to Pinocchio configuration indices for Go2 robot
// ========================================================================

void Robot_Model_Go2::assign_encoder_from_csv(Eigen::Vector<double, ENCODER_SIZE>& encoder, 
                                                 const Go2CSVData& csvData, 
                                                 int j) const {
    // Populate ENCODER with 29 configuration values for Go2 robot
    // Go2 has no floating base - all joints are revolute
    
    // // Left leg (6 DOF)
    // encoder(0) = csvData.left_hip_pitch[j];      // q[0]  → left_hip_pitch_joint
    // encoder(1) = csvData.left_hip_roll[j];       // q[1]  → left_hip_roll_joint
    // encoder(2) = csvData.left_hip_yaw[j];        // q[2]  → left_hip_yaw_joint
    // encoder(3) = csvData.left_knee[j];           // q[3]  → left_knee_joint
    // encoder(4) = csvData.left_ankle_pitch[j];    // q[4]  → left_ankle_pitch_joint
    // encoder(5) = csvData.left_ankle_roll[j];     // q[5]  → left_ankle_roll_joint
    
    // // Right leg (6 DOF)
    // encoder(6) = csvData.right_hip_pitch[j];     // q[6]  → right_hip_pitch_joint
    // encoder(7) = csvData.right_hip_roll[j];      // q[7]  → right_hip_roll_joint
    // encoder(8) = csvData.right_hip_yaw[j];       // q[8]  → right_hip_yaw_joint
    // encoder(9) = csvData.right_knee[j];          // q[9]  → right_knee_joint
    // encoder(10) = csvData.right_ankle_pitch[j];  // q[10] → right_ankle_pitch_joint
    // encoder(11) = csvData.right_ankle_roll[j];   // q[11] → right_ankle_roll_joint
    
    // // Waist (3 DOF)
    // encoder(12) = csvData.waist_yaw[j];          // q[12] → waist_yaw_joint
    // encoder(13) = csvData.waist_roll[j];         // q[13] → waist_roll_joint
    // encoder(14) = csvData.waist_pitch[j];        // q[14] → waist_pitch_joint
    
    // // Left arm (7 DOF)
    // encoder(15) = csvData.left_shoulder_pitch[j];  // q[15] → left_shoulder_pitch_joint
    // encoder(16) = csvData.left_shoulder_roll[j];   // q[16] → left_shoulder_roll_joint
    // encoder(17) = csvData.left_shoulder_yaw[j];    // q[17] → left_shoulder_yaw_joint
    // encoder(18) = csvData.left_elbow[j];           // q[18] → left_elbow_joint
    // encoder(19) = csvData.left_wrist_roll[j];      // q[19] → left_wrist_roll_joint
    // encoder(20) = csvData.left_wrist_pitch[j];     // q[20] → left_wrist_pitch_joint
    // encoder(21) = csvData.left_wrist_yaw[j];       // q[21] → left_wrist_yaw_joint
    
    // // Right arm (7 DOF)
    // encoder(22) = csvData.right_shoulder_pitch[j]; // q[22] → right_shoulder_pitch_joint
    // encoder(23) = csvData.right_shoulder_roll[j];  // q[23] → right_shoulder_roll_joint
    // encoder(24) = csvData.right_shoulder_yaw[j];   // q[24] → right_shoulder_yaw_joint
    // encoder(25) = csvData.right_elbow[j];          // q[25] → right_elbow_joint
    // encoder(26) = csvData.right_wrist_roll[j];     // q[26] → right_wrist_roll_joint
    // encoder(27) = csvData.right_wrist_pitch[j];    // q[27] → right_wrist_pitch_joint
    // encoder(28) = csvData.right_wrist_yaw[j];      // q[28] → right_wrist_yaw_joint
}

void Robot_Model_Go2::assign_encoderdot_from_csv(Eigen::Vector<double, ENCODERDOT_SIZE>& encoderdot, 
                                                     const Go2CSVData& csvData, 
                                                     int j) const {
    // ENCODERDOT - populate with 29 velocity values for Go2 robot
    // Go2 has no floating base - all velocities are joint velocities
    
    // // Left leg velocities (6 DOF)
    // encoderdot(0) = csvData.left_hip_pitch_vel[j];      // v[0]  → left_hip_pitch_joint
    // encoderdot(1) = csvData.left_hip_roll_vel[j];       // v[1]  → left_hip_roll_joint
    // encoderdot(2) = csvData.left_hip_yaw_vel[j];        // v[2]  → left_hip_yaw_joint
    // encoderdot(3) = csvData.left_knee_vel[j];           // v[3]  → left_knee_joint
    // encoderdot(4) = csvData.left_ankle_pitch_vel[j];    // v[4]  → left_ankle_pitch_joint
    // encoderdot(5) = csvData.left_ankle_roll_vel[j];     // v[5]  → left_ankle_roll_joint
    
    // // Right leg velocities (6 DOF)
    // encoderdot(6) = csvData.right_hip_pitch_vel[j];     // v[6]  → right_hip_pitch_joint
    // encoderdot(7) = csvData.right_hip_roll_vel[j];      // v[7]  → right_hip_roll_joint
    // encoderdot(8) = csvData.right_hip_yaw_vel[j];       // v[8]  → right_hip_yaw_joint
    // encoderdot(9) = csvData.right_knee_vel[j];          // v[9]  → right_knee_joint
    // encoderdot(10) = csvData.right_ankle_pitch_vel[j];  // v[10] → right_ankle_pitch_joint
    // encoderdot(11) = csvData.right_ankle_roll_vel[j];   // v[11] → right_ankle_roll_joint
    
    // // Waist velocities (3 DOF)
    // encoderdot(12) = csvData.waist_yaw_vel[j];          // v[12] → waist_yaw_joint
    // encoderdot(13) = csvData.waist_roll_vel[j];         // v[13] → waist_roll_joint
    // encoderdot(14) = csvData.waist_pitch_vel[j];        // v[14] → waist_pitch_joint
    
    // // Left arm velocities (7 DOF)
    // encoderdot(15) = csvData.left_shoulder_pitch_vel[j];  // v[15] → left_shoulder_pitch_joint
    // encoderdot(16) = csvData.left_shoulder_roll_vel[j];   // v[16] → left_shoulder_roll_joint
    // encoderdot(17) = csvData.left_shoulder_yaw_vel[j];    // v[17] → left_shoulder_yaw_joint
    // encoderdot(18) = csvData.left_elbow_vel[j];           // v[18] → left_elbow_joint
    // encoderdot(19) = csvData.left_wrist_roll_vel[j];      // v[19] → left_wrist_roll_joint
    // encoderdot(20) = csvData.left_wrist_pitch_vel[j];     // v[20] → left_wrist_pitch_joint
    // encoderdot(21) = csvData.left_wrist_yaw_vel[j];       // v[21] → left_wrist_yaw_joint
    
    // // Right arm velocities (7 DOF)
    // encoderdot(22) = csvData.right_shoulder_pitch_vel[j]; // v[22] → right_shoulder_pitch_joint
    // encoderdot(23) = csvData.right_shoulder_roll_vel[j];  // v[23] → right_shoulder_roll_joint
    // encoderdot(24) = csvData.right_shoulder_yaw_vel[j];   // v[24] → right_shoulder_yaw_joint
    // encoderdot(25) = csvData.right_elbow_vel[j];          // v[25] → right_elbow_joint
    // encoderdot(26) = csvData.right_wrist_roll_vel[j];     // v[26] → right_wrist_roll_joint
    // encoderdot(27) = csvData.right_wrist_pitch_vel[j];    // v[27] → right_wrist_pitch_joint
    // encoderdot(28) = csvData.right_wrist_yaw_vel[j];      // v[28] → right_wrist_yaw_joint
}

