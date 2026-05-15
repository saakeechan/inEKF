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
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "InEKF_estimator.hpp"
#include "../utility/csvParser.hpp"
#include "contact_detection.hpp"
#include "../utility/inekf_save_utils.hpp"
#include <filesystem>
#include <chrono>
#include <sstream>


using std::cout;  
using std::endl;

// This is just reading of the sensor data file and ground truth data file

void InEKFilter::LoadCSVData(const std::string& csv_filename) {
    CSVData = ParseCSV(csv_filename);
    std::cout << "Loaded CSV data with " << CSVData.timestamp.size() << " rows." << std::endl;
    csv_mode = true;
}

std::vector<double> InEKFilter::Variable_Contact_Cov(int frame_count){ // time is either 0 or 1 cause sliding window of 2 for filter; fix this for smoother later

    std::vector<double> contact_cov_array;

    for(int k=0; k<robot.num_legs; k++){

        double contact_cov = robot.cov_contact_const;

        //  only legs with actual contact get potentially adjusted covariance.
        //  Swing state legs get assingined a default value for consistent index purposes
        if(CONTACT_t[frame_count][k]){ 

            if(variable_contact_cov_mode){

                // Contact covariance increases exponentially with residual velocity which ideally should be zero
                // 10^( log10(cov_contact_const) + ||d_v|| * cov_amplifier )
                contact_cov = std::pow(10, log10(robot.cov_contact_const) + dv_norm[frame_count](k) * cov_amplifier );

                if( contact_cov > robot.cov_slip_const ){
                    contact_cov = robot.cov_slip_const;
                }

            }else if(slip_rejection_mode){

                // If the residual velocity exceeds the slip threshold, assign a high covariance
                if( (dv_norm[frame_count](k) > slip_threshold)){
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
    dv_norm.resize(SLIDING_WINDOW_SIZE);
    Bias_Acc_s.resize(SLIDING_WINDOW_SIZE);
    Bias_Gyro_s.resize(SLIDING_WINDOW_SIZE);
    CONTACT_t.resize(SLIDING_WINDOW_SIZE);
    HARD_CONTACT_t.resize(SLIDING_WINDOW_SIZE);
    SLIP_t.resize(SLIDING_WINDOW_SIZE); // think about changing to Slip_Contact_t
    IMU_Gyro.resize(SLIDING_WINDOW_SIZE);
    IMU_Acc.resize(SLIDING_WINDOW_SIZE);
    ENCODER.resize(SLIDING_WINDOW_SIZE);
    ENCODERDOT.resize(SLIDING_WINDOW_SIZE);
    MOTOR_TORQUES.resize(SLIDING_WINDOW_SIZE);
    MOTOR_TORQUES_CMD.resize(SLIDING_WINDOW_SIZE);
    CMD_V.resize(SLIDING_WINDOW_SIZE);
    torqueSumSquared.resize(SLIDING_WINDOW_SIZE);
    
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
        CMD_V[i] = Eigen::Vector3d::Zero();

        // Dynamic-size types based on robot model
        ENCODER[i] = Eigen::VectorXd::Zero(robot.encoder_size);
        ENCODERDOT[i] = Eigen::VectorXd::Zero(robot.encoderdot_size);
        MOTOR_TORQUES[i] = Eigen::VectorXd::Zero(robot.encoder_size);  // Same size as encoder
        MOTOR_TORQUES_CMD[i] = Eigen::VectorXd::Zero(robot.encoder_size);  // Same size as encoder
        
        d_v[i] = Eigen::VectorXd::Constant(3 * robot.num_legs, std::numeric_limits<double>::infinity());
        dv_norm[i] = Eigen::VectorXd::Zero(robot.num_legs);
        torqueSumSquared[i] = Eigen::VectorXd::Zero(robot.num_legs);  // 3 components per leg
        
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

    if(csv_mode){

        int idx = 0 + start_index; // Skip initial rows in CSV if needed

        // Initialize from CSV data + initial condition offset

        // Position from CSV data (pos_x, pos_y, pos_z)
        p0 << CSVData.pos_x[idx] + initial_condition(0),
              CSVData.pos_y[idx] + initial_condition(1),
              CSVData.pos_z[idx] + initial_condition(2);
        
        // Velocity from CSV data (vel_x, vel_y, vel_z)
        v0 << CSVData.vel_x[idx] + initial_condition(7),
              CSVData.vel_y[idx] + initial_condition(8),
              CSVData.vel_z[idx] + initial_condition(9);

        // Bias initialization
        bg0 << initial_condition(10), initial_condition(11), initial_condition(12);
        ba0 << initial_condition(13), initial_condition(14), initial_condition(15);

        // Convert quaternion from CSV to rotation matrix
        Eigen::Vector4d quat_csv;
        quat_csv << CSVData.quat_w[idx], CSVData.quat_i[idx], CSVData.quat_j[idx], CSVData.quat_k[idx];
        Eigen::Matrix3d R_csv = BasicFunctions_Estimator::Quaternion_to_Rotation_Matrix(quat_csv);

        // Get rotation offset from initial condition (quaternion: w, x, y, z)
        Eigen::Vector4d quat_offset = initial_condition.block(3, 0, 4, 1);
        Eigen::Matrix3d R_offset = BasicFunctions_Estimator::Quaternion_to_Rotation_Matrix(quat_offset);

        // Compose rotations: R0 = R_offset * R_csv (applies offset in body frame)
        R0 = R_offset * R_csv;

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

    
    // Load CSV settings if present
    if (config["textfile"]) {
        csv_mode = config["textfile"]["enabled"].as<bool>();
        // Load saved results folder path
        saved_results_folder = config["saved_results_folder"].as<std::string>();
        if (csv_mode) {
            std::string file_name = config["textfile"]["file_name"].as<std::string>();
            LoadCSVData(file_name);
            start_index = config["textfile"]["starting_point"].as<int>();
            time_length = config["textfile"]["time_length"].as<int>();
        }
    }
    else {
        csv_mode = false;
    }
    
    // Load contact model path
    if (config["Contact_estimation_NN"]) {
        contact_nn_enabled = config["Contact_estimation_NN"]["enabled"].as<bool>();
        if (contact_nn_enabled) {
            contact_model_path = config["Contact_estimation_NN"]["file_path"].as<std::string>();
            std::cout << "Contact model path set to: " << contact_model_path << std::endl;
            
            // Create ContactNN instance (robot is already loaded at this point)
            contact_nn = std::make_unique<ContactNN>(robot);
        }
    } else {
        std::cerr << "ERROR: Contact_estimation_NN section not found in config file!" << std::endl;
        std::cerr << "Please add 'Contact_estimation_NN: enabled: <true/false> file_path: <path_to_model>' to " << config_path << std::endl;
        throw std::runtime_error("Contact model configuration not found in initialization YAML config");

    }
    
    // Call the existing Initialize function with loaded parameters
    Initialize(dt_val, cov_val_setting, initial_condition, sliding_window_size);
}

// ============================================================================
// Neural Network Contact Model (delegates to ContactNN)
// ============================================================================

void InEKFilter::load_contact_model(const std::string& model_path) {
    // Create ContactNN instance if not already created
    if (!contact_nn) {
        contact_nn = std::make_unique<ContactNN>(robot);
    }
    // Delegate to ContactNN
    contact_nn->load_contact_model(model_path);
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
    // No-argument version: only for CSV mode
    if(!csv_mode) {
        std::cerr << "Error: new_measurement() with no arguments requires csv_mode == true" << std::endl;
        return;
    }
    
    int j = time_count + start_index;
    
    // IMU data from CSV
    IMU_Gyro[frame_count] << CSVData.gyro_body_x[j], CSVData.gyro_body_y[j], CSVData.gyro_body_z[j];
    IMU_Acc[frame_count] << CSVData.acc_body_x[j], CSVData.acc_body_y[j], CSVData.acc_body_z[j];

    if (robot.robot_name == "G1") {
    // Convert gravity-compensated body IMU to world frame with gravity added back
    // Eigen::Vector4d quat_gt;
    // quat_gt << CSVData.quat_w[j], CSVData.quat_i[j], CSVData.quat_j[j], CSVData.quat_k[j];
    // Eigen::Matrix3d R_gt = BasicFunctions_Estimator::Quaternion_to_Rotation_Matrix(quat_gt);
    // IMU_Acc[frame_count] = R_gt * IMU_Acc[frame_count] - gravity;
    }
    // IMU_Gyro[frame_count] = filter.getState().getRotation() * IMU_Gyro[frame_count];

    // cout << "Frame " << frame_count << " IMU Gyro: " << IMU_Gyro[frame_count].transpose() << endl;
    // cout << "Frame " << frame_count << " IMU Acc: " << IMU_Acc[frame_count].transpose() << endl;

    // ========================================================================
    // ROBOT-SPECIFIC ENCODER ASSIGNMENT
    // This section assigns encoder and encoder velocity data from CSV to 
    // the appropriate indices for Pinocchio FK.
    // 
    // When creating a new robot model:
    // 1. Map CSV joint data to the robot's joint configuration
    // 2. Ensure joint ordering matches your URDF/MJCF model
    // 3. Use robot.map_indices_to_encoders() for proper mapping
    // ========================================================================
    
    // Map CSV data to encoder vectors
    robot.map_indices_to_encoders(CSVData.encoders[j], ENCODER[frame_count], "encoder");
    robot.map_indices_to_encoders(CSVData.encoderdots[j], ENCODERDOT[frame_count], "encoderdot");
    
    // Get contact state from CSV
    CONTACT_t[frame_count] = CSVData.contact_state[j];
    
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
        
        // Extract Jacobian for this leg and compute measurement covariance
        Eigen::MatrixXd FK_Jacobian_full = robot.get_foot_jacobian(k+1);

        // Extract only the active encoder columns (removes base DOFs + unused joints)
        Eigen::MatrixXd FK_Jacobian = robot.extract_active_encoder_jacobian(FK_Jacobian_full);

        // std::cout << "Full FK Jacobian for leg " << k+1 << ":\n" << FK_Jacobian_full << std::endl;
        // std::cout << "Active FK Jacobian for leg " << k+1 << ":\n" << FK_Jacobian << std::endl;
        
        // Compute diagonal covariance from norm squared of Jacobian rows
        Eigen::Vector3d jac_norm_sq;
        for (int axis = 0; axis < 3; axis++) {
            jac_norm_sq(axis) = FK_Jacobian.row(axis).squaredNorm();
        }
        covariance.block(3,3, 3,3) = jac_norm_sq.asDiagonal() * robot.cov_enc_const;

        inekf::Kinematics frame(id, pose, covariance);
        measured_kinematics.push_back(frame);
    }

    filter.CorrectKinematics(measured_kinematics, true);
    

        // Store dv norms for this time step
        std::vector<double> dv_norms_current(robot.num_legs);

        // Always compute d_v and dv_norm from kinematics (not using NN for velocity prediction)
        for (int k=0; k<robot.num_legs; k++){
            d_v[frame_count].block(3*k,0,3,1) = filter.getState().getVelocity()
                    + filter.getState().getRotation()*robot.get_foot_velocity(k+1)
                    + filter.getState().getRotation()*BasicFunctions_Estimator::Hat_so3(IMU_Gyro[frame_count] - filter.getState().getGyroscopeBias())*robot.get_foot_position(k+1);
            dv_norm[frame_count](k) = d_v[frame_count].block(3*k,0,3,1).norm();
        }
        
        for (int k=0; k<robot.num_legs; k++){
        // Store the norm for this leg
            dv_norms_current[k] = dv_norm[frame_count](k);

            // if (k == 0) {
            //     std::cout << "Leg1 dv_norm: " << dv_norms_current[k] << "  contact: " << CONTACT_t[frame_count][k] << std::endl;
            // }

            SLIP_t[frame_count][k] = false;
            
            if (slip_rejection_mode == true && CONTACT_t[frame_count][k] == true &&
                dv_norm[frame_count](k) > slip_threshold){
                SLIP_t[frame_count][k] = true;
            }
            HARD_CONTACT_t[frame_count][k]  = CONTACT_t[frame_count][k] - SLIP_t[frame_count][k];
            // std::cout << "Frame Position " << k << ": " << robot.get_foot_position(k+1).transpose() << std::endl;
            // std::cout << "Frame Velocity " << k << ": " << robot.get_foot_velocity(k+1).transpose() << std::endl;
    }
    
    
    // Append dv norms to history
    dv_history.push_back(dv_norms_current);

    // Eigen::Vector3d w_IMU_world = filter.getState().getRotation() * (IMU_Gyro[frame_count] - filter.getState().getGyroscopeBias()); // assuming bias is in body frame.
    //     robot.transform_state_to_base(
    //     filter.getState().getRotation(),
    //     filter.getState().getPosition(),
    //     filter.getState().getVelocity(),
    //     w_IMU_world,
    //     robot_state.Rotation,
    //     robot_state.Position,
    //     robot_state.Velocity,
    //     robot_state.Angular_Velocity
    // );

    robot_state.Rotation = filter.getState().getRotation();
    robot_state.Position = filter.getState().getPosition();
    robot_state.Velocity = filter.getState().getVelocity();
    robot_state.Angular_Velocity = IMU_Gyro[frame_count]; // assuming bias is in body frame.

    
    // Populate remaining robot_state fields (sensor-specific and contact data)
    robot_state.Bias_Gyro = filter.getState().getGyroscopeBias();
    robot_state.Bias_Acc = filter.getState().getAccelerometerBias();
    robot_state.Contact = CONTACT_t[frame_count];
    robot_state.Hard_Contact = HARD_CONTACT_t[frame_count];
    robot_state.Slip = SLIP_t[frame_count];
    robot_state.d_v = d_v[frame_count];
    robot_state.dv_norm = dv_norm[frame_count];


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
        std::swap(MOTOR_TORQUES[0], MOTOR_TORQUES[1]);
        std::swap(d_v[0], d_v[1]);
        std::swap(dv_norm[0], dv_norm[1]);
    }
}

void InEKFilter::Onestep(const Eigen::VectorXd &imu_i, const Eigen::MatrixXd &motor_data_i, const std::vector<bool> &Contact_i){

    new_measurement(imu_i, motor_data_i, Contact_i); // read in new measurement and calculate rotation, velocity updates and slip states

    Propagate_and_Correct();
    // send_states(robot_state); // update current_state with latest estimates for external use (e.g., logging, visualization)

    if(filter.getState().getVelocity().norm()>10 && dt!=0){
       cout<<"Diverged!! from time "<<time_count<<endl;
        dt = 0;
    }

    // Keep frame_count in [0, 1] range for sliding window
    frame_count++;
    if(frame_count > 1) {
        frame_count = 1;
    }

    // Frame 1 information is now moved to frame 0
    sliding_window();


}
void InEKFilter::Onestep(const Eigen::VectorXd &imu_i, const Eigen::MatrixXd &motor_data_i,
     const Eigen::VectorXd &motor_torques_i, const Eigen ::VectorXd &motor_torques_cmd_i, const Eigen::VectorXd &Cmd_Velocity_i){

    new_measurement(imu_i, motor_data_i); // read motor and IMU data first

    if (contact_nn_enabled) {
        // Run NN detection at 0.02s intervals or on first frame
        if (time_count % static_cast<int>(0.02 / dt) == 0 || frame_count == 0) {

        // Store raw motor torques for NN feature extraction
        MOTOR_TORQUES[frame_count] = motor_torques_i;
        MOTOR_TORQUES_CMD[frame_count] = motor_torques_cmd_i; // Store commanded torques for potential future use/analysis
        CMD_V[frame_count] = Cmd_Velocity_i; // Store commanded velocities for potential future use/analysis

        // Compute sum of squared torques for each leg
        for (int leg = 0; leg < robot.num_legs; ++leg) {
            double sum_sq = 0.0;
            for (int joint_idx : robot.Leg_indices_map[leg]) {
                sum_sq += motor_torques_i(joint_idx) * motor_torques_i(joint_idx);
            }
            torqueSumSquared[frame_count](leg) = sum_sq;
        }

        // Calculate leg speeds by summing absolute encoder velocities for each leg
        Eigen::VectorXd leg_speeds(robot.num_legs);
        for (int i = 0; i < robot.num_legs; i++) {
            double leg_speed_sum = 0.0;
            for (int idx : robot.Leg_indices_map[i]) {
                leg_speed_sum += std::abs(ENCODERDOT[frame_count](idx));
            }
            leg_speeds(i) = leg_speed_sum;
        }

        // Extract NN features from current state (includes q, qd, acc, omega, p, v, tau)
        Eigen::VectorXd features = contact_nn->extract_nn_features(
            ENCODER[frame_count],
            ENCODERDOT[frame_count],
            IMU_Acc[frame_count],
            IMU_Gyro[frame_count],
            MOTOR_TORQUES[frame_count],
            MOTOR_TORQUES_CMD[frame_count],
            CMD_V[frame_count]
        );
        
        // Update sliding window buffer with new features
        contact_nn->update_nn_feature_buffer(features);
        
        // Predict contacts using neural network (returns contacts and foot velocities)
        auto [predicted_contacts, predicted_foot_velocities] = contact_nn->nn_predict_contact();
        
        // Implement hysteresis with probability thresholds (after sigmoid)
        for (int leg = 0; leg < robot.num_legs; ++leg) {
            double prob = predicted_contacts[leg]; // Extract probability value for this leg (post-sigmoid)
            if (prob > 0.5) {
                CONTACT_t[frame_count][leg] = true;
            } else if (prob < 0.5) {
                CONTACT_t[frame_count][leg] = false;
            } else if (frame_count > 0) {
                // Keep previous state (hysteresis zone)
                CONTACT_t[frame_count][leg] = CONTACT_t[frame_count - 1][leg];
            }
        }
        
        // Note: predicted_foot_velocities contains NN-predicted dv_norms for both legs
        // - predicted_foot_velocities(0) = left leg dv_norm
        // - predicted_foot_velocities(1) = right leg dv_norm
        // Currently not used; dv is computed kinematically in Propagate_and_Correct()

        dv_norm[frame_count] = predicted_foot_velocities; // Store NN-predicted foot velocities for potential future use/analysis
    }
    else {

        // Use previous frame's contact detection result
        CONTACT_t[frame_count] = CONTACT_t[frame_count - 1];
    }
    }
    else
    {
        
        // Extract leg torques into m-row vector
        Eigen::VectorXd leg_torques(robot.num_legs);
        for (int i = 0; i < robot.num_legs; i++) {
            // Sum squared torques for all motors in this leg
            double leg_torque_sum = 0.0;
            for (int idx : robot.Leg_indices_map[i]) {
                double torque = motor_torques_i(idx);
                leg_torque_sum += torque * torque;
            }
            leg_torques(i) = leg_torque_sum;
        }

        torqueSumSquared[frame_count] = leg_torques;

        // Calculate leg speeds by summing absolute encoder velocities for each leg
        Eigen::VectorXd leg_speeds(robot.num_legs);
        for (int i = 0; i < robot.num_legs; i++) {
            double leg_speed_sum = 0.0;
            for (int idx : robot.Leg_indices_map[i]) {
                leg_speed_sum += std::abs(motor_data_i(1, idx));  // Row 1 is velocities
            }
            leg_speeds(i) = leg_speed_sum;
        }
        
        // Detect contacts from motor torques and velocity residuals
        // Use previous dv if available (frame_count >= 1), otherwise use current (frame_count = 0)
        Eigen::VectorXd dv_for_detection = d_v[0]; // Use dv from frame 0 for detection, as it is the most recent after sliding window update
        CONTACT_t[frame_count] = ContactDetection::detect_contact(leg_torques, dv_for_detection, leg_speeds);
    }


    Propagate_and_Correct();

    // send_states(robot_state); // update current_state with latest estimates for external use (e.g., logging, visualization)

    if(filter.getState().getVelocity().norm()>10 && dt!=0){
       cout<<"Diverged!! from time "<<time_count<<endl;
        dt = 0;
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
    // No-argument version: uses CSV data directly (requires csv_mode == true)

    new_measurement(); // reads from CSV when csv_mode is true
    
    // Extract leg torques from CSV data
    int j = time_count + start_index;

    if(contact_nn_enabled) {
        
        // Apply time-division logic: only run NN every 0.02s or on first frame
        if (time_count % static_cast<int>(0.02 / dt) == 0 || frame_count == 0) {
            // Store raw motor torques for NN features
            MOTOR_TORQUES[frame_count] = Eigen::VectorXd::Zero(robot.encoder_size);
            for (int i = 0; i < robot.num_legs; i++) {
                for (int idx : robot.Leg_indices_map[i]) {
                    MOTOR_TORQUES[frame_count](idx) = CSVData.motor_torques[j][idx];
                }
            }

            // Store commanded motor torques for NN features
            MOTOR_TORQUES_CMD[frame_count] = Eigen::VectorXd::Zero(robot.encoder_size);
            for (int i = 0; i < robot.num_legs; i++) {
                for (int idx : robot.Leg_indices_map[i]) {
                    MOTOR_TORQUES_CMD[frame_count](idx) = CSVData.motor_torques_cmd[j][idx];
                }
            }

            // Store commanded velocity from CSV data
            CMD_V[frame_count] = Eigen::Vector3d(CSVData.CMD_V[j][0], CSVData.CMD_V[j][1], CSVData.CMD_V[j][2]);

            // Compute sum of squared torques for each leg
            for (int leg = 0; leg < robot.num_legs; ++leg) {
                double sum_sq = 0.0;
                for (int joint_idx : robot.Leg_indices_map[leg]) {
                    sum_sq += MOTOR_TORQUES[frame_count](joint_idx) * MOTOR_TORQUES[frame_count](joint_idx);
                }
                torqueSumSquared[frame_count](leg) = sum_sq; // Average squared torque per joint for this leg
            }


            // Calculate leg speeds by summing absolute encoder velocities for each leg
            Eigen::VectorXd leg_speeds(robot.num_legs);
            for (int i = 0; i < robot.num_legs; i++) {
                double leg_speed_sum = 0.0;
                for (int idx : robot.Leg_indices_map[i]) {
                    leg_speed_sum += std::abs(ENCODERDOT[frame_count](idx));
                }
                leg_speeds(i) = leg_speed_sum;
            }

            // Extract NN features from current state (includes q, qd, acc, omega, p, v, tau)
            Eigen::VectorXd features = contact_nn->extract_nn_features(
                ENCODER[frame_count],
                ENCODERDOT[frame_count],
                IMU_Acc[frame_count],
                IMU_Gyro[frame_count],
                MOTOR_TORQUES[frame_count],
                MOTOR_TORQUES_CMD[frame_count],
                CMD_V[frame_count]
            );
            
            // Update sliding window buffer with new features
            contact_nn->update_nn_feature_buffer(features);
            
            // Predict contacts using ONNX Runtime model (velocity output is ignored)
            auto [predicted_contacts, predicted_foot_velocities] = contact_nn->nn_predict_contact();
            // Implement hysteresis with probability thresholds (after sigmoid)
            // If probability is above 0.5 then contact is true, otherwise false
            for (int leg = 0; leg < robot.num_legs; ++leg) {
                double prob = predicted_contacts[leg]; // Extract probability value for this leg (post-sigmoid)
                if (prob > 0.5) {
                    CONTACT_t[frame_count][leg] = true;
                } else if (prob < 0.5) {
                    CONTACT_t[frame_count][leg] = false;
                } else if (frame_count > 0) {
                    // Keep previous state (hysteresis zone)
                    CONTACT_t[frame_count][leg] = CONTACT_t[frame_count - 1][leg];
                } else {
                    // For the first frame, if we're in the hysteresis zone, default to contact = true (conservative assumption)
                    CONTACT_t[frame_count][leg] = true;
                }
            }

            dv_norm[frame_count] = predicted_foot_velocities; // Store NN-predicted foot velocities for potential future use/analysis

            // Note: dv and dv_norm are computed in Propagate_and_Correct() using kinematics

            // // If torque sum squared exceeds threshold for any leg, set contact to true for that leg
            // for (int leg = 0; leg < robot.num_legs; ++leg) {
            //     if (torqueSumSquared[frame_count](leg) >= 2000) {
            //         CONTACT_t[frame_count][leg] = true;
            //     }
            // }

            // // If all leg speeds are low (< 1.0), assume standing (all legs in contact)
            // bool all_legs_slow = true;
            // for (int i = 0; i < robot.num_legs; i++) {
            //     if (leg_speeds(i) >= 1.0) {
            //         all_legs_slow = false;
            //         break;
            //     }
            // }
            // if (all_legs_slow) {
            //     for (int i = 0; i < robot.num_legs; i++) {
            //         CONTACT_t[frame_count][i] = true;
            //     }
            // }
        } else {
            // Use previous frame's contact detection result
            CONTACT_t[frame_count] = CONTACT_t[frame_count - 1];
            // Note: dv_norm is computed in Propagate_and_Correct() using kinematics
        }
    }
    else
    {
        // Extract leg torques from CSV data
        int j = time_count + start_index;
        Eigen::VectorXd leg_torques(robot.num_legs);
        for (int i = 0; i < robot.num_legs; i++) {
            // Sum squared torques for all motors in this leg
            double leg_torque_sum = 0.0;
            for (int idx : robot.Leg_indices_map[i]) {
                double torque = CSVData.motor_torques[j][idx];
                leg_torque_sum += torque * torque;
            }
            leg_torques(i) = leg_torque_sum;
        }

        Eigen::VectorXd leg_speeds = Eigen::VectorXd::Ones(robot.num_legs) * 1000; // Placeholder: set high leg speeds to avoid slip detection in CSV mode (since we don't have real motor velocity data)
        torqueSumSquared[frame_count] = leg_torques;
        Eigen::VectorXd dv_for_detection = d_v[0]; // Use dv from frame 0 for detection, as it is the most recent after sliding window update
        // CONTACT_t[frame_count] = ContactDetection::detect_contact(leg_torques, dv_for_detection, leg_speeds);
    }
    GroundTruthContact = CSVData.contact_state[j]; // For evaluation purposes, store ground truth contact from CSV
    
    // Print contact and d_v info for debugging
    std::cout << "Contacts: [" << CONTACT_t[frame_count][0] << ", " << CONTACT_t[frame_count][1] << "]";
    std::cout << " GT: [" << GroundTruthContact[0] << ", " << GroundTruthContact[1] << "]" << std::endl;

    // Compare detected contact with ground truth and update counters
    for (int k = 0; k < robot.num_legs; k++) {
        if (CONTACT_t[frame_count][k] == GroundTruthContact[k]) {
            contact_match_count[k]++;
        } else {
            contact_mismatch_count[k]++;
        }
    }
    total_comparisons++;

    Propagate_and_Correct();

    InEKF_SaveUtils::save_onestep_impl(*this, time_count); // save estimated and true states to SAVE_MATRIX
    
    if(filter.getState().getVelocity().norm()>10 && dt!=0){
       cout<<"Diverged!! from time "<<time_count<<endl;
        dt = 0;
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

void InEKFilter::getParameters(const std::string& urdf_path, const std::string& robot_name) {
    // Wrapper function that calls the utility
    extractParameters(urdf_path, robot_name);
}