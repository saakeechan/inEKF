/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   kinematics.cpp
 *  @author Ross Hartley
 *  @brief  Example of invariant filtering for contact-aided inertial navigation
 *  @date   September 25, 2018 my birthday
 **/

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "InEKF_estimator.hpp"
#include "../utility/TextFileDataStruct.hpp"
#include "contact_detection.hpp"


using std::cout;  
using std::endl;

// This is just reading of the sensor data file and ground truth data file

void InEKFilter::LoadTextFileData(const std::string& textfile_filename) {
    TextFileData.loadFromParser(textfile_filename);
    std::cout << "Loaded TextFile data with " << TextFileData.timestamp.size() << " rows." << std::endl;
    textfile_flag = true;
}

std::vector<double> InEKFilter::Variable_Contact_Cov(int frame_count){ // time is either 0 or 1 cause sliding window of 2 for filter; fix this for smoother later

    std::vector<double> contact_cov_array;

    for(int k=0; k<robot.num_legs; k++){

        double contact_cov = robot.cov_contact_const;

        //  only legs with actual contact get potentially adjusted covariance.
        //  Swing state legs get assingined a default value for consistent index purposes
        if(CONTACT_t[frame_count][k]){ 

            if(variable_contact_cov_mode){

                contact_cov = d_v[frame_count].block(3*k,0,3,1).norm();
                //Check below for how d_v is computed

                // Contact covariance increases exponentially with residual velocity which ideally should be zero
                // 10^( log10(cov_contact_const) + ||d_v|| * cov_amplifier )
                contact_cov = std::pow(10, log10(robot.cov_contact_const) + contact_cov*cov_amplifier );

                if( contact_cov > robot.cov_slip_const ){
                    contact_cov = robot.cov_slip_const;
                }

            }else if(slip_rejection_mode){

                // If the residual velocity exceeds the slip threshold, assign a high covariance
                if( (d_v[frame_count].block(3*k,0, 3,1).norm() > slip_threshold)){
                    contact_cov = robot.cov_slip_const;
                }else{
                    // otherwise assign the default contact covariance
                    contact_cov = robot.cov_contact_const;
                }

            }else{
                // if neither mode is on, just assign the default contact covariance
                contact_cov = robot.cov_contact_const;
            }

        }

        //push back is like append for each iteration of the leg
        contact_cov_array.push_back( contact_cov );

    }

    return contact_cov_array;
}

void InEKFilter::Initialize(double dt_arg, Eigen::Matrix<double,12,1> cov_val_setting, Eigen::Matrix<double, 16, 1> initial_condition, int sliding_window_size_arg)
{

    //  ---- Initialize invariant extended Kalman filter ----- //

    filter.Reset();

    frame_count = 0; // Initialize frame counter
    time_count = 0; // Initialize time counter
    
    // Set sliding window size and allocate buffers
    int SLIDING_WINDOW_SIZE = sliding_window_size_arg;
    std::cout << "Sliding window size set to: " << SLIDING_WINDOW_SIZE << std::endl;
    dt = dt_arg;
    
    // Initialize SAVE_MATRIX (moved to heap to avoid stack overflow)
    SAVE_MATRIX.resize(SAVEMAXDATA);
    for (int i = 0; i < SAVEMAXDATA; i++) {
        SAVE_MATRIX[i].resize(SAVEMAXCNT, 0.0);
    }
    
    // Resize all sliding window vectors
    Position_s.resize(SLIDING_WINDOW_SIZE);
    Rotation_s.resize(SLIDING_WINDOW_SIZE);
    Velocity_s.resize(SLIDING_WINDOW_SIZE);
    d_v.resize(SLIDING_WINDOW_SIZE);
    Bias_Acc_s.resize(SLIDING_WINDOW_SIZE);
    Bias_Gyro_s.resize(SLIDING_WINDOW_SIZE);
    CONTACT_t.resize(SLIDING_WINDOW_SIZE);
    HARD_CONTACT_t.resize(SLIDING_WINDOW_SIZE);
    SLIP_t.resize(SLIDING_WINDOW_SIZE); // think about changing to Slip_Contact_t
    IMU_Gyro.resize(SLIDING_WINDOW_SIZE);
    IMU_Acc.resize(SLIDING_WINDOW_SIZE);
    ENCODER.resize(SLIDING_WINDOW_SIZE);
    ENCODERDOT.resize(SLIDING_WINDOW_SIZE);
    
    // Initialize dv_history for tracking dv norms over time
    dv_history.clear();
    
    // Initialize contact detection accuracy tracking
    contact_match_count.resize(robot.num_legs, 0);
    contact_mismatch_count.resize(robot.num_legs, 0);
    total_comparisons = 0;
    GroundTruthContact.resize(robot.num_legs, false);
    
    // Initialize each element to the correct size
    for (int i = 0; i < SLIDING_WINDOW_SIZE; i++) {
        // Fixed-size types
        Position_s[i] = Eigen::Vector3d::Zero();
        Rotation_s[i] = Eigen::Matrix3d::Identity();
        Velocity_s[i] = Eigen::Vector3d::Zero();
        Bias_Gyro_s[i] = Eigen::Vector3d::Zero();
        Bias_Acc_s[i] = Eigen::Vector3d::Zero();
        IMU_Gyro[i] = Eigen::Vector3d::Zero();
        IMU_Acc[i] = Eigen::Vector3d::Zero();
        
        // Dynamic-size types based on robot model
        ENCODER[i] = Eigen::VectorXd::Zero(robot.encoder_size);
        ENCODERDOT[i] = Eigen::VectorXd::Zero(robot.encoderdot_size);
        d_v[i] = Eigen::VectorXd::Constant(3 * robot.num_legs, std::numeric_limits<double>::infinity());  // 3 components per leg
        
        // Contact state vectors - resize inner vector for each leg
        CONTACT_t[i].resize(robot.num_legs, false);
        HARD_CONTACT_t[i].resize(robot.num_legs, false);
        SLIP_t[i].resize(robot.num_legs, false);
    }
        
    robot.SetCovariances(cov_val_setting); // Initialize covariance of the 12x1 vector input


    std::string mode;
    if(variable_contact_cov_mode == true){
        mode="(CS" + BasicFunctions_Estimator::to_string_n_signficant_figures(cov_amplifier,2)+ ")";
    }else if(slip_rejection_mode == true){
        mode="(SR" + BasicFunctions_Estimator::to_string_n_signficant_figures(slip_threshold,2) + ")";
    }

    estimator_info = mode+"H(adj)";
    initial_info = "(" + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[0], 2) + "_" + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[1], 2) + "_"
                        + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[2], 2) + "_" + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[3], 2) + "_"
                        + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[4], 2) + "_" + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[5], 2) + "_"
                        + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[6], 2) + "_"+ BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[7], 2) + "_"
                        + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[8], 2) + "_"+ BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[9], 2) + "_"
                        + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[10], 2) + "_"+ BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[11], 2) + ")";

    //covariance setting, everything is diagonal and constant for each element
    // 15x15 covariance matrix for the state [R, v, p, bg, ba]
    Eigen::Matrix<double,15,15> ForP; 
    ForP.setZero();
    ForP.block(0,0,9,9) = robot.cov_prior_orientation_const * Eigen::MatrixXd::Identity(9,9); // orientation, position, velocity all have same prior covariance
    ForP.block(9,9,6,6) = robot.cov_prior_bias_gyro_const * Eigen::MatrixXd::Identity(6,6); // bias gyro, bias accel all have same prior covariance


    Eigen::Matrix3d R0;
    Eigen::Vector3d v0, p0, bg0, ba0;

    inekf::RobotState initial_state(ForP);

    if(textfile_flag){

        int idx = 0 + TextFile_Skip_Data; // To skip trash data in TextFile at the beginning

        // initialize from TextFile data + initial condition offset because we might want to start from non-zero state as described in the TextFile file

        // Position from TextFile data (pos_x, pos_y, pos_z)
        p0 << TextFileData.pos_x[idx] + initial_condition(0),
              TextFileData.pos_y[idx] + initial_condition(1),
              TextFileData.pos_z[idx] + initial_condition(2);
        
        // Velocity from TextFile data (vel_x, vel_y, vel_z)
        v0 << TextFileData.vel_x[idx] + initial_condition(7),
              TextFileData.vel_y[idx] + initial_condition(8),
              TextFileData.vel_z[idx] + initial_condition(9);

        // Bias initialization
        bg0 << initial_condition(10), initial_condition(11), initial_condition(12);
        ba0 << initial_condition(13), initial_condition(14), initial_condition(15);

        // Convert quaternion from TextFile to rotation matrix
        Eigen::Vector4d quat;
        quat << TextFileData.quat_w[idx], TextFileData.quat_i[idx], TextFileData.quat_j[idx], TextFileData.quat_k[idx];
        R0 = BasicFunctions_Estimator::Quaternion_to_Rotation_Matrix(quat);

        Vector3d Euler = BasicFunctions_Estimator::Rotation_to_EulerZYX(R0);
        // add initial condition offset
        Euler(0) = Euler(0) + initial_condition[3];
        Euler(1) = Euler(1) + initial_condition[4];
        Euler(2) = Euler(2) + initial_condition[5];
        R0 = BasicFunctions_Estimator::EulerZYX_to_R_bw(Euler);

    }else{ // Initialize for sensor readings

        Eigen::Vector4d initial_quaternion = initial_condition.block(3,0,4,1);
        R0=BasicFunctions_Estimator::Quaternion_to_Rotation_Matrix(initial_quaternion);
        v0.setZero(); // initial velocity
        p0=initial_condition.block(0,0,3,1);
        bg0.setZero(); // initial gyroscope bias
        ba0.setZero(); // initial accelerometer bias

    }


    initial_state.setRotation(R0);
    initial_state.setVelocity(v0);
    initial_state.setPosition(p0);
    initial_state.setGyroscopeBias(bg0);
    initial_state.setAccelerometerBias(ba0);

    // Initial State has initial states + prior covariance

    imu_measurement = Eigen::Matrix<double,6,1>::Zero();
    imu_measurement_prev = Eigen::Matrix<double,6,1>::Zero();

    // Initialize state covariance
    inekf::NoiseParams noise_params;
    noise_params.setGyroscopeNoise(sqrt(robot.cov_gyro_const));
    noise_params.setAccelerometerNoise(sqrt(robot.cov_acc_const));
    noise_params.setGyroscopeBiasNoise(sqrt(robot.cov_bias_gyro_const));
    noise_params.setAccelerometerBiasNoise(sqrt(robot.cov_bias_acc_const));
    noise_params.setContactNoise(sqrt(robot.cov_contact_const));
    noise_params.setLandmarkNoise(sqrt(robot.cov_contact_const));

    // Initialize filter
    filter.setState(initial_state);
    filter.setNoiseParams(noise_params);

   cout << "Noise parameters are initialized to: \n";
   cout << filter.getNoiseParams() << endl;
   cout << "Robot's state is initialized to: \n";
   cout << filter.getState() << endl;
   // Theta is bias of sensors 6x1 vector
}

void InEKFilter::InitializeFromYAML(const std::string& config_path) {
    // Load YAML configuration
    YAML::Node config = YAML::LoadFile(config_path);
    
    // // Load robot configuration path (only if not already loaded)
    // if (robot.num_legs == 0) {  // Check if robot config already loaded
    //     std::string robot_config_path = config["robot_config_path"].as<std::string>();
    //     load_robot_config(robot_config_path);
    // }
    
    // Load estimator parameters
    int sliding_window_size = config["sliding_window_size"].as<int>();
    double dt_val = config["dt"].as<double>();
    
    // Load covariance exponents
    auto cov_exp = config["covariance_exponents"];
    double gyro_exp = cov_exp["gyro"].as<double>();
    double acc_exp = cov_exp["acc"].as<double>();
    double slip_exp = cov_exp["slip"].as<double>();
    double contact_exp = cov_exp["contact"].as<double>();
    double encoder_exp = cov_exp["encoder"].as<double>();
    double bg_exp = cov_exp["bias_gyro"].as<double>();
    double ba_exp = cov_exp["bias_acc"].as<double>();
    double pri_ori_exp = cov_exp["prior_orientation"].as<double>();
    double pri_vel_exp = cov_exp["prior_velocity"].as<double>();
    double pri_pos_exp = cov_exp["prior_position"].as<double>();
    double pri_bg_exp = cov_exp["prior_bias_gyro"].as<double>();
    double pri_ba_exp = cov_exp["prior_bias_acc"].as<double>();
    
    // Build estimator covariances vector
    Eigen::Vector<double, 12> cov_val_setting;
    cov_val_setting << std::pow(10, gyro_exp), std::pow(10, acc_exp), 
                       std::pow(10, slip_exp), std::pow(10, contact_exp), 
                       std::pow(10, encoder_exp),
                       std::pow(10, bg_exp), std::pow(10, ba_exp),
                       std::pow(10, pri_ori_exp), std::pow(10, pri_vel_exp), 
                       std::pow(10, pri_pos_exp),
                       std::pow(10, pri_bg_exp), std::pow(10, pri_ba_exp);

    cov_info = "(" + BasicFunctions_Estimator::to_string_n_signficant_figures(gyro_exp, 2) +
               BasicFunctions_Estimator::to_string_n_signficant_figures(acc_exp, 2) +
               BasicFunctions_Estimator::to_string_n_signficant_figures(slip_exp, 2) +
               BasicFunctions_Estimator::to_string_n_signficant_figures(contact_exp, 2) +
               BasicFunctions_Estimator::to_string_n_signficant_figures(encoder_exp, 2)
               + " " + BasicFunctions_Estimator::to_string_n_signficant_figures(bg_exp, 2)
               + " " + BasicFunctions_Estimator::to_string_n_signficant_figures(pri_ori_exp, 2) +
               BasicFunctions_Estimator::to_string_n_signficant_figures(pri_bg_exp, 2) + ")";


    
    // Load initial condition
    auto ic = config["initial_condition"];
    auto pos = ic["position"].as<std::vector<double>>();
    auto ori = ic["orientation"].as<std::vector<double>>();
    auto vel = ic["velocity"].as<std::vector<double>>();
    auto bg = ic["bias_gyro"].as<std::vector<double>>();
    auto ba = ic["bias_acc"].as<std::vector<double>>();
    
    Eigen::Vector<double, 16> initial_condition;
    initial_condition << pos[0], pos[1], pos[2],
                         ori[0], ori[1], ori[2], ori[3],
                         vel[0], vel[1], vel[2],
                         bg[0], bg[1], bg[2],
                         ba[0], ba[1], ba[2];
    
    // Load slip rejection settings
    slip_rejection_mode = config["slip_rejection"]["enabled"].as<bool>();
    slip_threshold = config["slip_rejection"]["threshold"].as<double>();
    
    // Load variable contact covariance settings
    variable_contact_cov_mode = config["variable_contact_covariance"]["enabled"].as<bool>();
    cov_amplifier = config["variable_contact_covariance"]["amplifier"].as<double>();
    
    // Load TextFile settings if present
    if (config["textfile"]) {
        textfile_flag = config["textfile"]["enabled"].as<bool>();
        if (textfile_flag) {
            std::string file_name = config["textfile"]["file_name"].as<std::string>();
            LoadTextFileData(file_name);
            TextFile_Skip_Data = config["textfile"]["starting_point"].as<int>();
            time_length = config["textfile"]["time_length"].as<int>();
        }
    }
    
    // Call the existing Initialize function with loaded parameters
    Initialize(dt_val, cov_val_setting, initial_condition, sliding_window_size);
}

void InEKFilter::new_measurement(const Eigen::VectorXd &imu_i, const Eigen::MatrixXd &motor_data_i, const std::vector<bool> &Contact_i)
{
    // ========================================================================
    // ROBOT-SPECIFIC ENCODER ASSIGNMENT
    // imu_i: 6-element Eigen::VectorXd [omega_x, omega_y, omega_z, acc_x, acc_y, acc_z]
    // motor_data_i: 2 x num_joints Eigen::MatrixXd
    //               Row 0: joint positions
    //               Row 1: joint velocities
    // ========================================================================
    
    // Extract IMU data
    IMU_Gyro[frame_count] << imu_i(0), imu_i(1), imu_i(2);
    IMU_Acc[frame_count] << imu_i(3), imu_i(4), imu_i(5);
    
    // Extract encoder positions and velocities from matrix rows
    int num_joints = motor_data_i.cols();
    std::vector<double> encoder_data(num_joints);
    std::vector<double> encoderdot_data(num_joints);
    
    for (int i = 0; i < num_joints; ++i) {
        encoder_data[i] = motor_data_i(0, i);      // Row 0: positions
        encoderdot_data[i] = motor_data_i(1, i);   // Row 1: velocities
    }
    
    // Map directly to Eigen vectors (no conversion needed)
    robot.map_indices_to_encoders(encoder_data, ENCODER[frame_count], "encoder");
    robot.map_indices_to_encoders(encoderdot_data, ENCODERDOT[frame_count], "encoderdot");
    
    CONTACT_t[frame_count] = Contact_i;
    
    imu_measurement << IMU_Gyro[frame_count], IMU_Acc[frame_count];


    // Pinnocchio FK and Jacobians - compute forward kinematics and joint Jacobians as soon as you get new measurement
    robot.Forward_Kinematics(ENCODER[frame_count], ENCODERDOT[frame_count]);
    robot.compute_joint_jacobians();

}

void InEKFilter::new_measurement(const Eigen::VectorXd &imu_i, const Eigen::MatrixXd &motor_data_i)
{
    // ========================================================================
    // ROBOT-SPECIFIC ENCODER ASSIGNMENT
    // imu_i: 6-element Eigen::VectorXd [omega_x, omega_y, omega_z, acc_x, acc_y, acc_z]
    // motor_data_i: 2 x num_joints Eigen::MatrixXd
    //               Row 0: joint positions
    //               Row 1: joint velocities
    // Note: This version doesn't set contact info - caller must handle contacts
    // ========================================================================
    
    // Extract IMU data
    IMU_Gyro[frame_count] << imu_i(0), imu_i(1), imu_i(2);
    IMU_Acc[frame_count] << imu_i(3), imu_i(4), imu_i(5);
    
    // Extract encoder positions and velocities from matrix rows
    int num_joints = motor_data_i.cols();
    std::vector<double> encoder_data(num_joints);
    std::vector<double> encoderdot_data(num_joints);
    
    for (int i = 0; i < num_joints; ++i) {
        encoder_data[i] = motor_data_i(0, i);      // Row 0: positions
        encoderdot_data[i] = motor_data_i(1, i);   // Row 1: velocities
    }
    
    // Map directly to Eigen vectors (no conversion needed)
    robot.map_indices_to_encoders(encoder_data, ENCODER[frame_count], "encoder");
    robot.map_indices_to_encoders(encoderdot_data, ENCODERDOT[frame_count], "encoderdot");
    
    // Contact state will be set explicitly by the caller (via contact_detection_logic in Onestep)
    
    imu_measurement << IMU_Gyro[frame_count], IMU_Acc[frame_count];

    // Pinnocchio FK and Jacobians - compute forward kinematics and joint Jacobians as soon as you get new measurement
    robot.Forward_Kinematics(ENCODER[frame_count], ENCODERDOT[frame_count]);
    robot.compute_joint_jacobians();

}

void InEKFilter::new_measurement()
{
    // No-argument version: only for TextFile mode
    if(!textfile_flag) {
        std::cerr << "Error: new_measurement() with no arguments requires textfile_flag == true" << std::endl;
        return;
    }
    
    int j = time_count + TextFile_Skip_Data;
    
    // IMU data from TextFile
    IMU_Gyro[frame_count] << TextFileData.gyro_x[j], TextFileData.gyro_y[j], TextFileData.gyro_z[j];
    IMU_Acc[frame_count] << TextFileData.acc_x[j], TextFileData.acc_y[j], TextFileData.acc_z[j];

    // if (robot.robot_name == "G1") {
    // // Convert gravity-compensated body IMU to world frame with gravity added back
    // IMU_Acc[frame_count] = filter.getState().getRotation() * IMU_Acc[frame_count] - gravity;
    // }
    // IMU_Gyro[frame_count] = filter.getState().getRotation() * IMU_Gyro[frame_count];

    // cout << "Frame " << frame_count << " IMU Gyro: " << IMU_Gyro[frame_count].transpose() << endl;
    // cout << "Frame " << frame_count << " IMU Acc: " << IMU_Acc[frame_count].transpose() << endl;

    // ========================================================================
    // ROBOT-SPECIFIC ENCODER ASSIGNMENT
    // This section assigns encoder and encoder velocity data from TextFile to 
    // the appropriate indices for Pinocchio FK.
    // 
    // When creating a new robot model:
    // 1. Implement assign_encoder_from_TextFile() in your Robot_Model_XXX class
    // 2. Implement assign_encoderdot_from_TextFile() in your Robot_Model_XXX class
    // 3. These methods should map TextFile data to correct joint indices per your URDF/MJCF
    // ========================================================================
    
    // Map TextFile data to encoder vectors
    robot.map_indices_to_encoders(TextFileData.encoders[j], ENCODER[frame_count], "encoder");
    robot.map_indices_to_encoders(TextFileData.encoderdots[j], ENCODERDOT[frame_count], "encoderdot");
    
    // Get contact state from TextFile
    CONTACT_t[frame_count] = TextFileData.contact_state[j];
    
    // Consolidate IMU measurement
    imu_measurement << IMU_Gyro[frame_count], IMU_Acc[frame_count];
    
    // Compute forward kinematics and Jacobians
    robot.Forward_Kinematics(ENCODER[frame_count], ENCODERDOT[frame_count]);
    robot.compute_joint_jacobians();
    
    // std::cout << "Frame " << frame_count << " Encoder values: " << ENCODER[frame_count].transpose() << std::endl;
    // std::cout << "Frame " << frame_count << " EncoderDot values: " << ENCODERDOT[frame_count].transpose() << std::endl;

    // exit(0); // Exit after one measurement for debugging - remove this in actual use
}

void InEKFilter::Propagate_and_Correct()

{

    if(frame_count>0) {

        filter.Propagate(imu_measurement_prev, dt, Variable_Contact_Cov(frame_count - 1)); // propagate with previous imu measurement and dt and also the contact covariance for static foot assumption

    }

    // change imu measurement_prev after inputting to propagation
    imu_measurement_prev = imu_measurement;

    // // Build contact constraint list from slip-filtered hard contacts
    std::vector<pair<int,bool>> contactConstraints;
    for (int i=0; i<robot.num_legs; i++) {
        contactConstraints.emplace_back(i, CONTACT_t[frame_count][i]);
    }

    //cout<<"HC "<<HARD_CONTACT_t[frame_count]<<endl;
    filter.setContacts(contactConstraints);


    int id;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double,6,6> covariance;
    inekf::vectorKinematics measured_kinematics;
    

    // Build measured kinematics from batch results
    for (int k=0; k<robot.num_legs; k++) {
        id = k;
        pose.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
        pose.block<3,1>(0,3) = robot.get_foot_position(k+1);
        
        // Get active Jacobian for this leg (already extracts only active encoder columns)
        Eigen::MatrixXd FK_Jacobian = robot.get_foot_jacobian(k+1);
        
        // Compute diagonal covariance from norm squared of Jacobian rows
        Eigen::Vector3d jac_norm_sq;
        for (int axis = 0; axis < 3; axis++) {
            jac_norm_sq(axis) = FK_Jacobian.row(axis).squaredNorm();
        }
        covariance.block(3,3, 3,3) = jac_norm_sq.asDiagonal() * robot.cov_enc_const;

        inekf::Kinematics frame(id, pose, covariance);
        measured_kinematics.push_back(frame);
    }

    // exit(0); // Exit after building kinematics for debugging - remove this in actual use
    // Correct state using kinematic measurements

    filter.CorrectKinematics(measured_kinematics, true);
    

        // Store dv norms for this time step
        std::vector<double> dv_norms_current(robot.num_legs);

        // // DEBUG: Override filter velocity and rotation with ground truth for dv computation
        // int j_debug = time_count + TextFile_Skip_Data;
        // Eigen::Vector3d v_gt;
        // v_gt << TextFileData.vel_x[j_debug], TextFileData.vel_y[j_debug], TextFileData.vel_z[j_debug];
        // Eigen::Vector4d quat_gt;
        // quat_gt << TextFileData.quat_w[j_debug], TextFileData.quat_i[j_debug], TextFileData.quat_j[j_debug], TextFileData.quat_k[j_debug];
        // Eigen::Matrix3d R_gt = BasicFunctions_Estimator::Quaternion_to_Rotation_Matrix(quat_gt);

        for (int k=0; k<robot.num_legs; k++){
            
            d_v[frame_count].block(3*k,0,3,1) = filter.getState().getVelocity()
                    + filter.getState().getRotation()*robot.get_foot_velocity(k+1)
                    + filter.getState().getRotation()*BasicFunctions_Estimator::Hat_so3(IMU_Gyro[frame_count] - filter.getState().getGyroscopeBias())*robot.get_foot_position(k+1);

            // d_v[frame_count].block(3*k,0,3,1) = v_gt
            //         + R_gt * robot.get_foot_velocity(k+1)
            //         + R_gt * BasicFunctions_Estimator::Hat_so3(IMU_Gyro[frame_count])*robot.get_foot_position(k+1);
                    
            
        // Store the norm for this leg
            dv_norms_current[k] = d_v[frame_count].block(3*k,0, 3,1).norm();

            // if (k == 0) {
            //     std::cout << "Leg1 dv_norm: " << dv_norms_current[k] << "  contact: " << CONTACT_t[frame_count][k] << std::endl;
            // }

            SLIP_t[frame_count][k] = false;
            
            if (slip_rejection_mode == true && CONTACT_t[frame_count][k] == true &&
                d_v[frame_count].block(3*k,0, 3,1).norm() > slip_threshold){
                SLIP_t[frame_count][k] = true;
            }
            HARD_CONTACT_t[frame_count][k]  = CONTACT_t[frame_count][k] - SLIP_t[frame_count][k];
            // std::cout << "Frame Position " << k << ": " << robot.get_foot_position(k+1).transpose() << std::endl;
            // std::cout << "Frame Velocity " << k << ": " << robot.get_foot_velocity(k+1).transpose() << std::endl;
    }
    // std::cout << "Frame Velocity " << robot.get_foot_velocity(1).transpose() << std::endl;
    
    
    // Append dv norms to history
    dv_history.push_back(dv_norms_current);

}

void InEKFilter::send_states(ROBOT_STATES &state_){ 

    //Sending Estimated States----------------------------------------------------------------------------

    state_.Position = filter.getState().getPosition();

    state_.Velocity = filter.getState().getVelocity();
    state_.Bias_Gyro = filter.getState().getGyroscopeBias();
    state_.Bias_Acc = filter.getState().getAccelerometerBias();
    state_.Rotation= filter.getState().getRotation();

    state_.Contact = CONTACT_t[frame_count];
    state_.Hard_Contact = HARD_CONTACT_t[frame_count];
    state_.Slip = SLIP_t[frame_count];
    state_.d_v = d_v[frame_count];

}

void InEKFilter::SAVE_onestep(int save_index){

    if(save_index<SAVEMAXCNT)
    {
        if(textfile_flag){
            //Sending True States----------------------------------------------------------------------------
            int j = time_count + TextFile_Skip_Data;

            // Build rotation matrix from TextFile quaternion data
            Eigen::Vector4d quat_true;
            quat_true << TextFileData.quat_w[j], TextFileData.quat_i[j], TextFileData.quat_j[j], TextFileData.quat_k[j];
            Eigen::Matrix3d temp_R = BasicFunctions_Estimator::Quaternion_to_Rotation_Matrix(quat_true);

            Eigen::Vector3d temp_eul = BasicFunctions_Estimator::Rotation_to_EulerZYX(temp_R);

            for(int i=0;i<3;i++)
            {
                for(int k=0;k<3;k++)
                {
                    SAVE_MATRIX[idx_TRUE_Rotation+i*3+k][save_index] = temp_R(i,k);
                }
            }


            Eigen::Vector3d v_true;
            Eigen::Vector<double,3> dv_true;
            Eigen::Matrix3d R_true;

            for(int i=0;i<robot.num_legs;i++){

                v_true << TextFileData.vel_x[j], TextFileData.vel_y[j], TextFileData.vel_z[j];
                
                // Build R_true from TextFile quaternion data (same as temp_R)
                Eigen::Vector4d quat_true_loop;
                quat_true_loop << TextFileData.quat_w[j], TextFileData.quat_i[j], TextFileData.quat_j[j], TextFileData.quat_k[j];
                R_true = BasicFunctions_Estimator::Quaternion_to_Rotation_Matrix(quat_true_loop);
                
                dv_true = v_true
                          + R_true * robot.get_foot_velocity(i+1)
                          + R_true * BasicFunctions_Estimator::Hat_so3(IMU_Gyro[frame_count])*robot.get_foot_position(i+1);


                for(int k=0; k<3; k++){

                    SAVE_MATRIX[idx_ESTIMATED_dv + 3 * i + k][save_index] = d_v[frame_count](3*i + k);
                    SAVE_MATRIX[idx_TRUE_dv + 3 * i + k][save_index] = dv_true(k);
                }
            }



            // Save position, velocity, biases, and RPY
            for(int i=0;i<3;i++)
            {
                SAVE_MATRIX[idx_TRUE_Position+i][save_index] = TextFileData.pos_x[j] * (i == 0) + TextFileData.pos_y[j] * (i == 1) + TextFileData.pos_z[j] * (i == 2);
                SAVE_MATRIX[idx_TRUE_Velocity+i][save_index] = TextFileData.vel_x[j] * (i == 0) + TextFileData.vel_y[j] * (i == 1) + TextFileData.vel_z[j] * (i == 2);
                SAVE_MATRIX[idx_TRUE_Bias_Gyro+i][save_index] = 0;
                SAVE_MATRIX[idx_TRUE_Bias_Acc+i][save_index] = 0;
                SAVE_MATRIX[idx_TRUE_rpy+i][save_index] = temp_eul(i);
            }


            // Save contact states
            std::vector<bool> temp_contact = TextFileData.contact_state[j];
            
            for(int i=0;i<robot.num_legs;i++)
            {
                SAVE_MATRIX[idx_TRUE_Contact+i][save_index] = temp_contact[i];
                SAVE_MATRIX[idx_TRUE_Slip+i][save_index] = SLIP_t[frame_count][i];
                SAVE_MATRIX[idx_TRUE_Hard_Contact+i][save_index] = HARD_CONTACT_t[frame_count][i];
            }

        }else{

            // I dont know nwhat to do for sensor derived state estimation
        }



        for(int i=0;i<robot.num_legs;i++)
        {
            SAVE_MATRIX[idx_ESTIMATED_Contact+i][save_index] = CONTACT_t[frame_count][i];
            SAVE_MATRIX[idx_ESTIMATED_Slip+i][save_index] = SLIP_t[frame_count][i];
            SAVE_MATRIX[idx_ESTIMATED_Hard_Contact+i][save_index] = HARD_CONTACT_t[frame_count][i];

        }



        Eigen::Vector3d temp_eul = BasicFunctions_Estimator::Rotation_to_EulerZYX(filter.getState().getRotation());
        for(int i=0;i<3;i++)
        {
            SAVE_MATRIX[idx_ESTIMATED_Position+i][save_index] = filter.getState().getPosition()(i);
            SAVE_MATRIX[idx_ESTIMATED_Velocity+i][save_index] = filter.getState().getVelocity()(i);
            SAVE_MATRIX[idx_ESTIMATED_Bias_Gyro+i][save_index] = filter.getState().getGyroscopeBias()(i);
            SAVE_MATRIX[idx_ESTIMATED_Bias_Acc+i][save_index] = filter.getState().getAccelerometerBias()(i);
            SAVE_MATRIX[idx_ESTIMATED_rpy+i][save_index] = temp_eul(i);
        }

        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                SAVE_MATRIX[idx_ESTIMATED_Rotation+i*3+j][save_index] = filter.getState().getRotation()(i,j);
            }
        }

    }
    else
    {
        save_index = SAVEMAXCNT-1;
        cout<<"over Max_SAVE_cnt!!"<<endl;
    }
}

// where previous data is moved to index 0 and new data is moved to index 1
void InEKFilter::sliding_window(){

    if (frame_count>0){
        std::swap(HARD_CONTACT_t[0],HARD_CONTACT_t[1]);
        std::swap(CONTACT_t[0],CONTACT_t[1]);
        std::swap(SLIP_t[0],SLIP_t[1]);
        std::swap(IMU_Gyro[0],IMU_Gyro[1]);
        std::swap(IMU_Acc[0],IMU_Acc[1]);
        std::swap(ENCODER[0],ENCODER[1]);
        std::swap(ENCODERDOT[0],ENCODERDOT[1]);
        std::swap(d_v[0], d_v[1]);
    }
}

void InEKFilter::writeSavedData(){

    // Build filename efficiently
    std::string filename = "../result/pinnochio_inekf/" + estimator_info + "_" + cov_info + "_" + 
                          initial_info + "_" + file_info + "x" + std::to_string(time_count) + "_1.txt";

    // Use C++ file stream with RAII and error checking
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "Error: Failed to open file: " << filename << std::endl;
        return;
    }

    // Write data with proper formatting
    for(int i = 0; i < time_count; i++) {
        for(int j = 0; j < SAVEMAXDATA; j++) {
            outfile << SAVE_MATRIX[j][i] << "\t";
        }
        outfile << "\n";
    }

    // File automatically closed by destructor
    cout << filename << endl;
    cout << "Saving to a file done" << endl;
    
    // // Print contact detection accuracy statistics
    // if (total_comparisons > 0) {
    //     cout << "\n=== Contact Detection Accuracy ==="<< endl;
    //     cout << "Total comparisons: " << total_comparisons << endl;
    //     for (int k = 0; k < robot.num_legs; k++) {
    //         int total_leg = contact_match_count[k] + contact_mismatch_count[k];
    //         double accuracy = (total_leg > 0) ? (100.0 * contact_match_count[k] / total_leg) : 0.0;
    //         cout << "Leg " << (k+1) << ": Matches=" << contact_match_count[k] 
    //              << ", Mismatches=" << contact_mismatch_count[k]
    //              << ", Accuracy=" << std::fixed << std::setprecision(2) << accuracy << "%" << endl;
    //     }
    // }
    
    // Write dv history to CSV
    // writeDvHistory();

}

void InEKFilter::Onestep(const Eigen::VectorXd &imu_i, const Eigen::MatrixXd &motor_data_i, const std::vector<bool> &Contact_i){

    new_measurement(imu_i, motor_data_i, Contact_i); // read in new measurement and calculate rotation, velocity updates and slip states


    Propagate_and_Correct();
    send_states(robot_state); // update current_state with latest estimates for external use (e.g., logging, visualization)

    if(filter.getState().getVelocity().norm()>10 && dt!=0){
    //    cout<<"Diverged!! from time "<<time_count<<endl;
        // dt = 0;
    }

    // Keep frame_count in [0, 1] range for sliding window
    frame_count++;
    if(frame_count > 1) {
        frame_count = 1;
    }

    // Frame 1 information is now moved to frame 0
    sliding_window();


}
void InEKFilter::Onestep(const Eigen::VectorXd &imu_i, const Eigen::MatrixXd &motor_data_i, const Eigen::VectorXd &motor_torques_i){

    new_measurement(imu_i, motor_data_i); // read motor and IMU data first

    // Extract leg torques into m-row vector
    Eigen::VectorXd leg_torques(robot.num_legs);
    for (int i = 0; i < robot.num_legs; i++) {
        // Sum torques for all motors in this leg
        double leg_torque_sum = 0.0;
        for (int idx : robot.Torque_indices_map[i]) {
            leg_torque_sum += std::abs(motor_torques_i(idx));
        }
        leg_torques(i) = leg_torque_sum;
    }
    
    // Detect contacts from motor torques and velocity residuals
    // Use previous dv if available (frame_count >= 1), otherwise use current (frame_count = 0)
    Eigen::VectorXd dv_for_detection = d_v[0]; // Use dv from frame 0 for detection, as it is the most recent after sliding window update
    CONTACT_t[frame_count] = ContactDetection::detect_contact(leg_torques, dv_for_detection);

    Propagate_and_Correct();

    send_states(robot_state); // update current_state with latest estimates for external use (e.g., logging, visualization)

    if(filter.getState().getVelocity().norm()>10 && dt!=0){
    //    cout<<"Diverged!! from time "<<time_count<<endl;
        // dt = 0;
    }

    // Keep frame_count in [0, 1] range for sliding window
    frame_count++;
    if(frame_count > 1) {
        frame_count = 1;
    }

    // Frame 1 information is now moved to frame 0
    sliding_window();


}
void InEKFilter::Onestep(){    
    // No-argument version: uses TextFile data directly (requires textfile_flag == true)

    new_measurement(); // reads from TextFile when textfile_flag is true

    // // Create zero leg torques vector (only checking dv-based detection)
    // Eigen::VectorXd leg_torques = Eigen::VectorXd::Zero(robot.num_legs);
    
    // Eigen::VectorXd dv_for_detection = (frame_count < 1) ? d_v[frame_count] : d_v[frame_count - 1];
    
    // ContactDetection::detect_contact(leg_torques, dv_for_detection);

    
    // Extract leg torques from TextFile data
    int j = time_count + TextFile_Skip_Data;
    Eigen::VectorXd leg_torques(robot.num_legs);
    for (int i = 0; i < robot.num_legs; i++) {
        // Sum squared torques for all motors in this leg
        double leg_torque_sum = 0.0;
        for (int idx : robot.Torque_indices_map[i]) {
            double torque = TextFileData.motor_torques[j][idx];
            leg_torque_sum += torque * torque;
        }
        leg_torques(i) = leg_torque_sum;
    }
    
    Eigen::VectorXd dv_for_detection = d_v[0]; // Use dv from frame 0 for detection, as it is the most recent after sliding window update
    CONTACT_t[frame_count] = ContactDetection::detect_contact(leg_torques, dv_for_detection);

    // GroundTruthContact = TextFileData.contact_state[j]; // For evaluation purposes, store ground truth contact from TextFile

    // // Compare detected contact with ground truth and update counters
    // for (int k = 0; k < robot.num_legs; k++) {
    //     if (CONTACT_t[frame_count][k] == GroundTruthContact[k]) {
    //         contact_match_count[k]++;
    //     } else {
    //         contact_mismatch_count[k]++;
    //     }
    // }
    // total_comparisons++;

    Propagate_and_Correct();

    SAVE_onestep(time_count); // save estimated and true states to SAVE_MATRIX

    // send_states(current_state); // update current_state with latest estimates for external use (e.g., logging, visualization)

    
    if(filter.getState().getVelocity().norm()>10 && dt!=0){
    //    cout<<"Diverged!! from time "<<time_count<<endl;
        // dt = 0;
    }
    
    // Keep frame_count in [0, 1] range for sliding window
    frame_count++;
    if(frame_count > 1) {
        frame_count = 1;
    }

    time_count++;
    
    // Frame 1 information is now moved to frame 0
    sliding_window();
}

void InEKFilter::writeDvHistory() {
    std::string filename = robot.robot_name + "_dv_values.csv";

    std::ofstream outfile(filename);
    
    if (!outfile.is_open()) {
        std::cerr << "Error: Failed to open file: " << filename << std::endl;
        return;
    }
    
    // Write header row
    for (int k = 0; k < robot.num_legs; k++) {
        outfile << "Leg_" << (k+1);
        if (k < robot.num_legs - 1) {
            outfile << ",";
        }
    }
    outfile << "\n";
    
    // Write data rows (each time step)
    for (size_t t = 0; t < dv_history.size(); t++) {
        for (int k = 0; k < robot.num_legs; k++) {
            outfile << dv_history[t][k];
            if (k < robot.num_legs - 1) {
                outfile << ",";
            }
        }
        outfile << "\n";
    }
    
    cout << "dv history saved to " << filename << " (" << dv_history.size() << " time steps, " << robot.num_legs << " legs)" << endl;
}

void InEKFilter::getParameters(const std::string& urdf_path, const std::string& robot_name) {
    // Wrapper function that calls the utility
    extractParameters(urdf_path, robot_name);
}