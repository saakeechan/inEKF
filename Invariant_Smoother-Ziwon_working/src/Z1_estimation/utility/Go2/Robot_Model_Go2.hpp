//
// Created by Saaketh 01/05/26
// Go2-Specific Robot Model with Pinocchio
//

#ifndef SRC_ROBOT_MODEL_Go2_HPP
#define SRC_ROBOT_MODEL_Go2_HPP

#include <cmath>
#include <cassert>
#include <cstring>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <stdio.h>
#include <iostream>
#include "BasicFunctions_Estimator.hpp"
#include <vector>

// Pinocchio includes
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/mjcf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

using std::cout;
using std::endl;

// Forward declaration
struct Go2CSVData;

class Robot_Model_Go2{

public:
    Robot_Model_Go2();
    ~Robot_Model_Go2();
    
    // ========================================================================
    // Go2-SPECIFIC CONFIGURATION
    // ========================================================================
    
    // Number of legs (4 for quadruped Go2)
    static constexpr int NUM_LEGS = 4;
    
    // Number of contact points (same as NUM_LEGS for Go2)
    static constexpr int NUM_CONTACTS = NUM_LEGS;
    
    // Robot model file path
    std::string robot_model_path = "../unitree_ros/robots/go2_description/urdf/go2_description.urdf";

    // Contact point frame IDs for Go2: Left foot=16 (left_ankle_roll_link), Right foot=32 (right_ankle_roll_link)
    std::vector<int> contact_point_frame_ids = {15, 35, 59, 79};
    
    // Encoder configuration for Go2 robot:
    // - nq (configuration dimension) = 12
    // - nv (velocity dimension) = 12 
    // These match the Pinocchio model dimensions loaded from the URDF file
    static constexpr int ENCODER_SIZE = 12;     // nq - configuration space dimension
    static constexpr int ENCODERDOT_SIZE = 12;  // nv - velocity space dimension

    // ========================================================================
    // END Go2-SPECIFIC CONFIGURATION
    // ========================================================================
    
    // Pinocchio model and data
    pinocchio::Model pin_model;
    pinocchio::Data pin_data;
    std::vector<pinocchio::FrameIndex> foot_frame_ids;  // Contact point frame IDs
    bool use_pinocchio = true;
    
    // Load XML and initialize Pinocchio model
    void load_xml(const std::string& xml_path, const std::vector<int>& contact_frame_ids = {});

    
    Eigen::Vector3d Gravity;
    Eigen::Vector3d IMU2BD;

    // Covariance constants - Gets overridden in Initialize function of InEKF_estimator
    double cov_gyro_const               = 1e-4;
    double cov_acc_const                = 1e-2;
    double cov_slip_const               = 1e-3;
    double cov_contact_const            = 1e-3;
    double cov_enc_const                = 1e-4;
    double cov_bias_gyro_const          = 1e-6;
    double cov_bias_acc_const           = 1e-6;
    double cov_prior_orientation_const  = 1e-3;
    double cov_prior_velocity_const     = 1e-3;
    double cov_prior_position_const     = 1e-3;
    double cov_prior_bias_gyro_const    = 1e-6;
    double cov_prior_bias_acc_const     = 1e-6;

    // Covariance matrices
    Eigen::Matrix<double,3,3>   Covariance_Gyro;
    Eigen::Matrix<double,3,3>   Covariance_Acc;
    Eigen::Matrix<double,3,3>   Covariance_Contact;
    Eigen::Matrix<double,3,3>   Covariance_Slip;
    Eigen::Matrix<double,3,3>   Covariance_Encoder;
    Eigen::Matrix<double,3,3>   Covariance_Bias_Gyro;
    Eigen::Matrix<double,3,3>   Covariance_Bias_Acc;
    Eigen::Matrix<double,3,3>   Covariance_Prior_Orientation;
    Eigen::Matrix<double,3,3>   Covariance_Prior_Velocity;
    Eigen::Matrix<double,3,3>   Covariance_Prior_Position;
    Eigen::Matrix<double,3,3>   Covariance_Prior_Bias_Gyro;
    Eigen::Matrix<double,3,3>   Covariance_Prior_Bias_Acc;

    // Square root information matrices
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Gyro;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Acc;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Contact;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Slip;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Encoder;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Bias_Gyro;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Bias_Acc;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Prior_Orientation;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Prior_Velocity;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Prior_Position;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Prior_Bias_Gyro;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Prior_Bias_Acc;

    void Covariance_Reset(Eigen::Matrix<double,12,1> cov_val_setting);
    
    bool variable_contact_cov_mode = false;
    double cov_amplifier = 0;
    bool slip_rejection_mode = false;
    double slip_threshold = 0;

    ///////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////     Kinematics Functions (Pinocchio)  ///////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////

    void Forward_Kinematics(const Eigen::VectorXd &q_leg, const Eigen::VectorXd &q_dot_leg) const;
    
    // Retrieve foot position after calling Forward_Kinematics
    Eigen::Vector3d get_foot_position(int i_leg) const;
    
    // Retrieve foot Jacobian after calling Forward_Kinematics
    Eigen::MatrixXd get_foot_jacobian(int i_leg) const;

    Eigen::Vector3d get_foot_velocity(int i_leg) const;

    void compute_joint_jacobians() const;

    // ========================================================================
    // Go2-SPECIFIC ENCODER ASSIGNMENT
    // Maps CSV data to Pinocchio configuration/velocity vectors
    // ========================================================================
    
    // Assign encoder values from CSV data for Go2 robot
    void assign_encoder_from_csv(Eigen::Vector<double, ENCODER_SIZE>& encoder, const Go2CSVData& csvData, int time_index) const;
    
    // Assign encoder velocity values from CSV data for Go2 robot
    void assign_encoderdot_from_csv(Eigen::Vector<double, ENCODERDOT_SIZE>& encoderdot, const Go2CSVData& csvData, int time_index) const;

private:

};


#endif //SRC_ROBOT_MODEL_Go2_HPP
