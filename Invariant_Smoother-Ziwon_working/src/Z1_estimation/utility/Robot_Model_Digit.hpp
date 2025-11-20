//
// Created by Saaketh 11/17/25
// Digit Robot Model with Pinocchio
//

#ifndef SRC_ROBOT_MODEL_DIGIT_HPP
#define SRC_ROBOT_MODEL_DIGIT_HPP

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

class Robot_Model_Digit {

public:
    Robot_Model_Digit();
    ~Robot_Model_Digit();

    // Digit has 2 legs
    int leg_no = 2;
    
    // Pinocchio model and data
    pinocchio::Model pin_model;
    pinocchio::Data pin_data;
    std::vector<pinocchio::FrameIndex> foot_frame_ids;  // Left and Right foot frames
    bool use_pinocchio = true;
    
    // Load XML and initialize Pinocchio model
    void load_xml(const std::string& xml_path);
    
    double dt = 0;
    // double BASIC_HEIGHT = 0.95;  // Digit standing height //
    // this is not used anywhere as well

    
    enum leg_indexing{
        LEFT = 1,
        RIGHT = 2
    };

    enum leg_contact_point_frame_ids{
        LEFT_CONTACT_POINT_FRAME_ID = 31,
        RIGHT_CONTACT_POINT_FRAME_ID = 71
    };

    Eigen::Vector3d Gravity;
    Eigen::Vector3d IMU2BD;

    double magnitude = 1.0;

    double cov_gyro_const               = 1e-4;
    double cov_acc_const                = 1e-2;

    double cov_slip_const         = 1e-3;
    double cov_contact_const        = 1e-3;
    double cov_enc_const                = 1e-4;


    double cov_bias_gyro_const          = 1e-6;
    double cov_bias_acc_const           = 1e-6;
    //31888

    double cov_prior_orientation_const  = 1e-3;
    double cov_prior_velocity_const     = 1e-3;
    double cov_prior_position_const     = 1e-3;

    double cov_prior_bias_gyro_const    = 1e-6;
    double cov_prior_bias_acc_const     = 1e-6;






    Eigen::Matrix<double,3,3>   Covariance_Gyro;
    Eigen::Matrix<double,3,3>   Covariance_Acc;
    Eigen::Matrix<double,3,3>   Covariance_Contact;
    Eigen::Matrix<double,3,3>   Covariance_Slip;


    Eigen::Matrix<double,3,3> Covariance_Encoder;

    Eigen::Matrix<double,3,3> Covariance_Bias_Gyro;
    Eigen::Matrix<double,3,3> Covariance_Bias_Acc;

    Eigen::Matrix<double,3,3> Covariance_Prior_Orientation;
    Eigen::Matrix<double,3,3> Covariance_Prior_Velocity;
    Eigen::Matrix<double,3,3> Covariance_Prior_Position;
    Eigen::Matrix<double,3,3> Covariance_Prior_Bias_Gyro;
    Eigen::Matrix<double,3,3> Covariance_Prior_Bias_Acc;


    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Gyro             ;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Acc              ;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Contact      ;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Slip      ;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Encoder          ;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Bias_Gyro        ;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Bias_Acc         ;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Prior_Orientation;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Prior_Velocity   ;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Prior_Position   ;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Prior_Bias_Gyro  ;
    Eigen::Matrix<double,3,3> SQRT_INFO_Covariance_Prior_Bias_Acc   ;


    void Covariance_Reset(Eigen::Matrix<double,12,1> cov_val_setting);
    std::vector<double> Variable_Contact_Cov(Eigen::Matrix<bool,-1,1> Contact, Eigen::Matrix<double,-1,1> dv);
    bool variable_contact_cov_mode=false;
    double cov_amplifier=0;
    bool slip_rejection_mode=false;
    double slip_threshold =0 ;

    // double long_term_v_threshold;
    // double long_term_a_threshold;


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


private:

};


#endif //SRC_ROBOT_MODEL_DIGIT_HPP