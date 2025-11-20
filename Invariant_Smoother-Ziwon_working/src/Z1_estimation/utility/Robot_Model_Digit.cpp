//
// Created by Saaketh 11/17/25
// Digit Robot Model Implementation with Pinocchio
//

#include "Robot_Model_Digit.hpp"
#include <stdio.h>
#include <iostream>
#include <pinocchio/parsers/mjcf.hpp>


Robot_Model_Digit::Robot_Model_Digit() {
    Gravity << 0.0, 0.0, -9.80665;
    IMU2BD.setZero();
    
    // Load Digit XML model
    if (use_pinocchio) {
        load_xml("../digit_mujoco-main/src/digit_mujoco/models/flat/digit-v3-flat.xml");
    }
}

Robot_Model_Digit::~Robot_Model_Digit() {

}

void Robot_Model_Digit::load_xml(const std::string& xml_path) {
    try {
        pinocchio::mjcf::buildModel(xml_path, pin_model);
        pin_data = pinocchio::Data(pin_model);
        
        std::cout << "Successfully loaded Digit XML from: " << xml_path << std::endl;
        std::cout << "Number of joints: " << pin_model.njoints << std::endl;
        std::cout << "Number of frames: " << pin_model.nframes << std::endl;
        
        // Find foot frame IDs
        foot_frame_ids.clear();
        foot_frame_ids.resize(2);
        
        foot_frame_ids[0] = LEFT_CONTACT_POINT_FRAME_ID;
        foot_frame_ids[1] = RIGHT_CONTACT_POINT_FRAME_ID;

        
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
    

    // here have to manually assign q values to match pinocchio model

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
    
    if (i_leg < 1 || i_leg > 2) {
        std::cerr << "Error: Invalid leg index " << i_leg << ". Must be 1 (LEFT) or 2 (RIGHT)" << std::endl;
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
    
    if (i_leg < 1 || i_leg > 2) {
        std::cerr << "Error: Invalid leg index " << i_leg << ". Must be 1 (LEFT) or 2 (RIGHT)" << std::endl;
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
    
    if (i_leg < 1 || i_leg > 2) {
        std::cerr << "Error: Invalid leg index " << i_leg << ". Must be 1 (LEFT) or 2 (RIGHT)" << std::endl;
        return Eigen::MatrixXd::Zero(3, pin_model.nv);
    }
    
    // Compute frame Jacobian in World frame
    // Add comments
    // This is efficient because Forward_Kinematics() already computed the kinematics
    Eigen::MatrixXd J_full = Eigen::MatrixXd::Zero(6, pin_model.nv);
    pinocchio::getFrameJacobian(
        pin_model,
        const_cast<pinocchio::Data&>(pin_data),
        foot_frame_ids[i_leg - 1],
        pinocchio:: WORLD,
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

// check accuracy of FK and Jacobian later


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

std::vector<double>
Robot_Model_Digit::Variable_Contact_Cov(Eigen::Matrix<bool, -1, 1> Contact, Eigen::Matrix<double, -1, 1> dv) {

    std::vector<double> contact_cov_array;
    contact_cov_array.clear();

    for (int k = 0; k < leg_no; k++) {

        double contact_cov = cov_contact_const;

        if (Contact(k)) {
            if (variable_contact_cov_mode) {
                contact_cov = dv.block(3 * k, 0, 3, 1).norm();
                contact_cov = std::pow(10, log10(cov_contact_const) + contact_cov * cov_amplifier);
                if (contact_cov > cov_slip_const) {
                    contact_cov = cov_slip_const;
                }
            } else if (slip_rejection_mode) {
                if ((dv.block(3 * k, 0, 3, 1).norm() > slip_threshold)) {
                    contact_cov = cov_slip_const;
                } else {
                    contact_cov = cov_contact_const;
                }
            } else {
                contact_cov = cov_contact_const;
            }
        }
        contact_cov_array.push_back(contact_cov);
    }

    //cout<<"contact_cov is "<<endl<<contact_cov_array[0]<<", "<<contact_cov_array[1]<<", "<<contact_cov_array[2]<<", "<<contact_cov_array[3]<<", "<<endl;

    return contact_cov_array;
}



