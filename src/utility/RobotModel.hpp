//
// Created by Saaketh 01/06/26
// Unified Robot Model with YAML Configuration Support
//

#ifndef SRC_ROBOT_MODEL_HPP
#define SRC_ROBOT_MODEL_HPP

#include <cmath>
#include <cassert>
#include <cstring>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <yaml-cpp/yaml.h>

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

/**
 * @brief Unified Robot Model class that supports multiple robot types via YAML configuration
 * 
 */
class RobotModel {

public:
    /**
     * @brief Default constructor - requires manual configuration loading
     */
    RobotModel();
    
    ~RobotModel();
    
    /**
     * @brief Load robot configuration from YAML file
     * @param config_file Path to the YAML configuration file
     * @return true if successful, false otherwise
     */
    bool load_config(const std::string& config_file);
    
    // ========================================================================
    // ROBOT CONFIGURATION (loaded from YAML)
    // ========================================================================
    
    std::string robot_name;          // e.g., "digit", "g1"
    
    int num_legs;                    // Number of legs
    
    std::string robot_model_path;    // Path to URDF/MJCF file
    std::string model_format;        // "urdf" or "mjcf"
    std::vector<int> contact_point_frame_ids;  // Frame IDs for contact points
    int imu_frame_id;                // Frame ID for IMU in Pinocchio model
    
    int encoder_size;                // Configuration space dimension (nq) - auto-set from model
    int encoderdot_size;             // Velocity space dimension (nv) - auto-set from model
    
    std::vector<int> Encoder_indices_map;     // Indices for mapping TextFile data to encoder vector
    std::vector<int> Encoderdot_indices_map;  // Indices for mapping TextFile data to encoderdot vector
    std::vector<std::vector<int>> Leg_indices_map;  // 2D array: each row = indices for one leg
    
    // ========================================================================
    // PINOCCHIO MODEL
    // ========================================================================
    
    pinocchio::Model pin_model;
    pinocchio::Data pin_data;
    std::vector<pinocchio::FrameIndex> foot_frame_ids;
    
    /**
     * @brief Load XML/URDF/MJCF and initialize Pinocchio model
     * @param Path Path to the model file
     * @param contact_frame_ids Frame IDs for contact points
     */
    void load_robot_model(const std::string& xml_path, const std::vector<int>& contact_frame_ids = {});
    
    // ========================================================================
    // FRAME TRANSFORMATIONS
    // ========================================================================
    
    // ========================================================================
    // COVARIANCE PARAMETERS
    // ========================================================================
    
    // Covariance constants (loaded from YAML)
    double cov_gyro_const;
    double cov_acc_const;
    double cov_slip_const;
    double cov_contact_const;
    double cov_enc_const;
    double cov_bias_gyro_const;
    double cov_bias_acc_const;
    double cov_prior_orientation_const;
    double cov_prior_velocity_const;
    double cov_prior_position_const;
    double cov_prior_bias_gyro_const;
    double cov_prior_bias_acc_const;

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

    /**
     * @brief Reset covariance matrices based on provided values
     * @param cov_val_setting 12x1 vector of covariance values
     */
    void SetCovariances(Eigen::Matrix<double,12,1> cov_val_setting);
    
    // // Contact detection parameters
    // bool variable_contact_cov_mode;
    // double cov_amplifier;
    // bool slip_rejection_mode;
    // double slip_threshold;

    // ========================================================================
    // KINEMATICS FUNCTIONS (Pinocchio)
    // ========================================================================

    /**
     * @brief Compute forward kinematics for all joints and frames
     * @param q_leg Configuration vector (encoder values)
     * @param q_dot_leg Velocity vector (encoder velocities)
     */
    void Forward_Kinematics(const Eigen::VectorXd &q_leg, const Eigen::VectorXd &q_dot_leg) const;
    
    /**
     * @brief Get foot position after calling Forward_Kinematics
     * @param i_leg Leg index (1-indexed)
     * @return 3D position vector of the foot
     */
    Eigen::Vector3d get_foot_position(int i_leg) const;
    
    /**
     * @brief Get foot Jacobian after calling Forward_Kinematics
     * @param i_leg Leg index (1-indexed)
     * @return Jacobian matrix for the foot frame
     */
    Eigen::MatrixXd get_foot_jacobian(int i_leg) const;

    /**
     * @brief Get foot velocity after calling Forward_Kinematics
     * @param i_leg Leg index (1-indexed)
     * @return 3D velocity vector of the foot
     */
    Eigen::Vector3d get_foot_velocity(int i_leg) const;

    /**
     * @brief Compute joint Jacobians for all joints
     */
    void compute_joint_jacobians() const;

    // ========================================================================
    // ENCODER DATA MAPPING
    // ========================================================================
    
    /**
     * @brief Map TextFile data to encoder vector using configured indices from YAML
     * @param TextFile_data Vector of TextFile sensor data values
     * @param output_vector Output encoder vector - must be pre-allocated
     * @param encoder_type Type of encoder data: "encoder" for positions or "encoderdot" for velocities
     */
    void map_indices_to_encoders(const std::vector<double>& motor_data, Eigen::VectorXd& output_vector, const std::string& encoder_type) const;

    /**
     * @brief Extract active encoder columns from full Jacobian matrix
     * 
     * This method:
     * 1. Removes the first 6 columns (floating base DOFs: 3 translation + 3 rotation)
     * 2. Extracts only columns corresponding to joints with actual encoder measurements
     *    (based on TextFile_encoderdot_indices from YAML config)
     * 
     * Use this to get the Jacobian mapping from encoder measurement errors to 
     * end-effector position errors for measurement covariance computation.
     * 
     * @param full_jacobian Full Jacobian matrix from Pinocchio (3xN or 6xN where N includes base+joints)
     * @return Reduced Jacobian with only active encoder columns (3 x num_active_encoders)
     */
    Eigen::MatrixXd extract_active_encoder_jacobian(const Eigen::MatrixXd& full_jacobian) const;

    /**
     * @brief Transform all state quantities from World-IMU frame to World-Base frame
     * @param R_IMU_world Rotation matrix representing IMU orientation in world frame (from filter)
     * @param p_IMU_world Position of IMU in world frame (from filter)
     * @param v_IMU_world Linear velocity of IMU in world frame (from filter)
     * @param w_IMU_world Angular velocity of IMU in world frame (bias-corrected gyro)
     * @param R_base_World Output: Base orientation in world frame
     * @param p_base_World Output: Base position in world frame
     * @param v_base_World Output: Base linear velocity in world frame
     * @param w_base_World Output: Base angular velocity in world frame
     */
    void transform_state_to_base(
        const Eigen::Matrix3d& R_IMU_world,
        const Eigen::Vector3d& p_IMU_world,
        const Eigen::Vector3d& v_IMU_world,
        const Eigen::Vector3d& w_IMU_world,
        Eigen::Matrix3d& R_base_World,
        Eigen::Vector3d& p_base_World,
        Eigen::Vector3d& v_base_World,
        Eigen::Vector3d& w_base_World
    ) const;

private:
    YAML::Node config;  // Stores the loaded YAML configuration
    
    /**
     * @brief Load covariance parameters from YAML config
     */
    void load_covariance_params();
    
    /**
     * @brief Initialize covariance matrices with default values
     */
    void initialize_covariances();
};

#endif // SRC_ROBOT_MODEL_HPP
