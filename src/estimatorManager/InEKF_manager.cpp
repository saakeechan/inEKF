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
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "InEKF_manager.hpp"
#include "../utility/csvParser.hpp"
#include "contact_detection.hpp"
#include "../utility/inekf_save_utils.hpp"
#include <filesystem>
#include <chrono>
#include <sstream>
#include <stdexcept>


using std::cout;  
using std::endl;

// This is just reading of the sensor data file and ground truth data file

void INEKFilter_manager::LoadCSVData(const std::string& csv_filename_arg) {
    CSVData = ParseCSV(csv_filename_arg);
    std::cout << "Loaded CSV data with " << CSVData.timestamp.size() << " rows." << std::endl;
    csv_mode = true;
}

std::vector<double> INEKFilter_manager::Variable_Contact_Cov(int frame_index_arg) {
    std::vector<double> covariances(robot.model.num_legs, cov_contact_const);

    for (int leg = 0; leg < robot.model.num_legs; ++leg) {
        if (!CONTACT_t[frame_index_arg].contact[leg]) {
            continue;
        }

        const double residual = CONTACT_t[frame_index_arg].dv_norm(leg);
        if (std::isnan(residual)) continue;

        if (variable_contact_cov_mode) {
            covariances[leg] = std::min(
                cov_contact_const * std::pow(10.0, residual * cov_amplifier),
                cov_slip_const);
        } else if (slip_rejection_mode && residual > slip_threshold) {
            covariances[leg] = cov_slip_const;
        }
    }

    return covariances;
}

void INEKFilter_manager::Initialize(double dt_arg, Eigen::Matrix<double,12,1> cov_val_setting_arg, Eigen::Matrix<double, 16, 1> initial_condition_arg, int sliding_window_size_arg)
{

    if (!cov_val_setting_arg.allFinite() || (cov_val_setting_arg.array() <= 0.0).any()) {
        throw std::invalid_argument("Estimator covariances must be finite and positive");
    }

    //  ---- Initialize invariant extended Kalman filter ----- //

    filter.Reset();

    if (sliding_window_size_arg < 2) {
        throw std::invalid_argument("Sliding window size must be at least two frames");
    }
    sliding_window_size = sliding_window_size_arg;
    frame_count = sliding_window_size - 2; // Initial state occupies the previous-frame slot
    time_count = 0; // Initialize time counter
    imu_history_initialized = false;
    
    // Set sliding window size and allocate buffers
    std::cout << "Sliding window size set to: " << sliding_window_size << std::endl;
    dt = dt_arg;
    
    // Initialize SAVE_MATRIX (moved to heap to avoid stack overflow)
    SAVE_MATRIX.resize(SAVEMAXDATA);
    for (int i = 0; i < SAVEMAXDATA; i++) {
        SAVE_MATRIX[i].assign(SAVEMAXCNT, 0.0);
    }
    
    // Resize all sliding window vectors
    Position_s.resize(sliding_window_size);
    Rotation_s.resize(sliding_window_size);
    Velocity_s.resize(sliding_window_size);
    Bias_Acc_s.resize(sliding_window_size);
    Bias_Gyro_s.resize(sliding_window_size);
    CONTACT_t.resize(sliding_window_size);
    IMU_Gyro.resize(sliding_window_size);
    IMU_Acc.resize(sliding_window_size);
    ENCODER.resize(sliding_window_size);
    ENCODERDOT.resize(sliding_window_size);
    MOTOR_TORQUES.resize(sliding_window_size);
    
    InEKF_SaveUtils::initialize_logging(*this);

    // Initialize the latest NN foot-velocity outputs.
    dv_norm_nn.resize(0);
    NN_foot_velocity_world = Eigen::VectorXd::Zero(3 * robot.model.num_legs);
    NN_foot_velocity_variance_world = Eigen::VectorXd::Zero(3 * robot.model.num_legs); // Look later

    // Initialize each element to the correct size and default values
    for (int i = 0; i < sliding_window_size; i++) {
        // Fixed-size types
        Position_s[i] = Eigen::Vector3d::Zero();
        Rotation_s[i] = Eigen::Matrix3d::Identity();
        Velocity_s[i] = Eigen::Vector3d::Zero();
        Bias_Gyro_s[i] = Eigen::Vector3d::Zero();
        Bias_Acc_s[i] = Eigen::Vector3d::Zero();
        IMU_Gyro[i] = Eigen::Vector3d::Zero();
        IMU_Acc[i] = Eigen::Vector3d::Zero();

        // Dynamic-size types based on robot model
        ENCODER[i] = Eigen::VectorXd::Zero(robot.model.encoder_size);
        ENCODERDOT[i] = Eigen::VectorXd::Zero(robot.model.encoderdot_size);
        MOTOR_TORQUES[i] = Eigen::VectorXd::Zero(robot.model.encoder_size);  // Same size as encoder
        
        CONTACT_t[i].reset(robot.model.num_legs, std::numeric_limits<double>::infinity());
    }
        
    // Store effective noise settings in the estimator; leave robot defaults unchanged.
    cov_gyro_const = cov_val_setting_arg(0);
    cov_acc_const = cov_val_setting_arg(1);
    cov_slip_const = cov_val_setting_arg(2);
    cov_contact_const = cov_val_setting_arg(3);
    cov_enc_const = cov_val_setting_arg(4);
    cov_bias_gyro_const = cov_val_setting_arg(5);
    cov_bias_acc_const = cov_val_setting_arg(6);
    cov_prior_orientation_const = cov_val_setting_arg(7);
    cov_prior_velocity_const = cov_val_setting_arg(8);
    cov_prior_position_const = cov_val_setting_arg(9);
    cov_prior_bias_gyro_const = cov_val_setting_arg(10);
    cov_prior_bias_acc_const = cov_val_setting_arg(11);


    std::string mode;
    if(variable_contact_cov_mode == true){
        mode="(CS" + BasicFunctions_Estimator::to_string_n_signficant_figures(cov_amplifier,2)+ ")";
    }else if(slip_rejection_mode == true){
        mode="(SR" + BasicFunctions_Estimator::to_string_n_signficant_figures(slip_threshold,2) + ")";
    }

    estimator_info = mode+"H(adj)";
    initial_info = "(" + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition_arg[0], 2) + "_" + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition_arg[1], 2) + "_"
                        + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition_arg[2], 2) + "_" + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition_arg[3], 2) + "_"
                        + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition_arg[4], 2) + "_" + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition_arg[5], 2) + "_"
                        + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition_arg[6], 2) + "_"+ BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition_arg[7], 2) + "_"
                        + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition_arg[8], 2) + "_"+ BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition_arg[9], 2) + "_"
                        + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition_arg[10], 2) + "_"+ BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition_arg[11], 2) + ")";

    //covariance setting, everything is diagonal and constant for each element
    // 15x15 covariance matrix for the state [R, v, p, bg, ba]
    Eigen::Matrix<double,15,15> ForP; 
    ForP.setZero();
    ForP.block<3,3>(0,0) = cov_prior_orientation_const * Eigen::Matrix3d::Identity();
    ForP.block<3,3>(3,3) = cov_prior_velocity_const * Eigen::Matrix3d::Identity();
    ForP.block<3,3>(6,6) = cov_prior_position_const * Eigen::Matrix3d::Identity();
    ForP.block<3,3>(9,9) = cov_prior_bias_gyro_const * Eigen::Matrix3d::Identity();
    ForP.block<3,3>(12,12) = cov_prior_bias_acc_const * Eigen::Matrix3d::Identity();


    Eigen::Matrix3d R0;
    Eigen::Vector3d v0, p0, bg0, ba0;

    inekf::RobotState initial_state(ForP);

    int init_csv_index = start_index;
    if(csv_mode){

        int idx = init_csv_index; // Skip initial rows in CSV if needed

        // Initialize from CSV data + initial condition offset

        // Position from CSV data (pos_x, pos_y, pos_z)
        p0 << CSVData.pos_x[idx] + initial_condition_arg(0),
              CSVData.pos_y[idx] + initial_condition_arg(1),
              CSVData.pos_z[idx] + initial_condition_arg(2);
        
        // Velocity from CSV data (vel_x, vel_y, vel_z)
        v0 << CSVData.vel_x[idx] + initial_condition_arg(7),
              CSVData.vel_y[idx] + initial_condition_arg(8),
              CSVData.vel_z[idx] + initial_condition_arg(9);

        // Bias initialization
        bg0 << initial_condition_arg(10), initial_condition_arg(11), initial_condition_arg(12);
        ba0 << initial_condition_arg(13), initial_condition_arg(14), initial_condition_arg(15);

        // Convert quaternion from CSV to rotation matrix
        Eigen::Vector4d quat_csv;
        quat_csv << CSVData.quat_w[idx], CSVData.quat_i[idx], CSVData.quat_j[idx], CSVData.quat_k[idx];
        Eigen::Matrix3d R_csv = BasicFunctions_Estimator::Quaternion_to_Rotation_Matrix(quat_csv);

        // Get rotation offset from initial condition (quaternion: w, x, y, z)
        Eigen::Vector4d quat_offset = initial_condition_arg.block(3, 0, 4, 1);
        Eigen::Matrix3d R_offset = BasicFunctions_Estimator::Quaternion_to_Rotation_Matrix(quat_offset);

        // Left multiplication applies the orientation offset in the world frame.
        R0 = R_offset * R_csv;

    }else{ // Initialize for sensor readings

        Eigen::Vector4d initial_quaternion = initial_condition_arg.block(3,0,4,1);
        R0=BasicFunctions_Estimator::Quaternion_to_Rotation_Matrix(initial_quaternion);
        v0=initial_condition_arg.block(7,0,3,1);
        p0=initial_condition_arg.block(0,0,3,1);
        bg0=initial_condition_arg.block(10,0,3,1);
        ba0=initial_condition_arg.block(13,0,3,1);

    }


    initial_state.setRotation(R0);
    initial_state.setVelocity(v0);
    initial_state.setPosition(p0);
    initial_state.setGyroscopeBias(bg0);
    initial_state.setAccelerometerBias(ba0);

    // Initial State has initial states + prior covariance

    imu_measurement = Eigen::Matrix<double,6,1>::Zero();

    // Initialize state covariance
    inekf::NoiseParams noise_params;
    noise_params.setGyroscopeNoise(sqrt(cov_gyro_const));
    noise_params.setAccelerometerNoise(sqrt(cov_acc_const));
    noise_params.setGyroscopeBiasNoise(sqrt(cov_bias_gyro_const));
    noise_params.setAccelerometerBiasNoise(sqrt(cov_bias_acc_const));
    noise_params.setContactNoise(sqrt(cov_contact_const));
    noise_params.setLandmarkNoise(sqrt(cov_contact_const));

    // Initialize filter
    filter.setState(initial_state);
    filter.setNoiseParams(noise_params);

    if (csv_mode) { // chheck whether the first forward kineamtics and contacts really matter for the correction step, or if it is just for the initial state
        int init_frame = frame_count;
        const Eigen::VectorXd initial_imu = ReadCSVIMU(init_csv_index);
        IMU_Gyro[init_frame] = initial_imu.head<3>();
        IMU_Acc[init_frame] = initial_imu.tail<3>();
        imu_history_initialized = true;

        ENCODER[init_frame] = robot.model.map_indices_to_encoders(CSVData.encoders[init_csv_index], "encoder");
        ENCODERDOT[init_frame] = robot.model.map_indices_to_encoders(CSVData.encoderdots[init_csv_index], "encoderdot");
        CONTACT_t[init_frame].contact = CSVData.contact_state[init_csv_index];

        robot.model.Forward_Kinematics(ENCODER[init_frame], ENCODERDOT[init_frame]);
        robot.model.compute_joint_jacobians();

        std::vector<std::pair<int, bool>> contactConstraints;
        inekf::vectorKinematics measured_kinematics;

        for (int k = 0; k < robot.model.num_legs; k++) {
            contactConstraints.emplace_back(k, CONTACT_t[init_frame].contact[k]);

            Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
            pose.block<3,1>(0,3) = robot.model.get_foot_position(k);

            Eigen::Matrix<double,6,6> covariance = Eigen::Matrix<double,6,6>::Zero();
            covariance.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * cov_enc_const;
            const Eigen::MatrixXd jacobian = robot.model.get_foot_jacobian(k);
            covariance.block<3,3>(3,3) =
                cov_enc_const * jacobian * jacobian.transpose();

            measured_kinematics.emplace_back(k, pose, covariance);
        }

        filter.setContacts(contactConstraints);
        filter.CorrectKinematics(kinematics_mode, measured_kinematics, false); // Here, true or false doesn't matter, since the filter has just been reset and contacts will newly be added. The correction step will be applied after the first propagation step anyways
    }

    auto initialized_state = filter.getState();
    store_state_history(initialized_state);
    send_states(robot.state);
    InEKF_SaveUtils::save_onestep_impl(*this, 0);

   cout << "Noise parameters are initialized to: \n";
   cout << filter.getNoiseParams() << endl;
   cout << "Robot's state is initialized to: \n";
   cout << filter.getState() << endl;
   // Theta is bias of sensors 6x1 vector

   frame_count = sliding_window_size - 1;
}

void INEKFilter_manager::InitializeFromYAML(const std::string& config_path_arg) {
    // Load YAML configuration
    YAML::Node config = YAML::LoadFile(config_path_arg);
    
    // Load estimator parameters
    const int sliding_window_size_arg = config["sliding_window_size"].as<int>();
    double dt_val = config["dt"].as<double>();

    kinematics_mode = config["kinematics_mode"].as<std::string>();
    
    // Start from robot defaults; supplied YAML covariance exponents override them.
    auto cov_exp = config["covariance_exponents"];
    double gyro_exp = cov_exp && cov_exp["gyro"]
        ? cov_exp["gyro"].as<double>() : std::log10(robot.model.cov_gyro_const);
    double acc_exp = cov_exp && cov_exp["acc"]
        ? cov_exp["acc"].as<double>() : std::log10(robot.model.cov_acc_const);
    double slip_exp = cov_exp && cov_exp["slip"]
        ? cov_exp["slip"].as<double>() : std::log10(robot.model.cov_slip_const);
    double contact_exp = cov_exp && cov_exp["contact"]
        ? cov_exp["contact"].as<double>() : std::log10(robot.model.cov_contact_const);
    double encoder_exp = cov_exp && cov_exp["encoder"]
        ? cov_exp["encoder"].as<double>() : std::log10(robot.model.cov_enc_const);
    double bg_exp = cov_exp && cov_exp["bias_gyro"]
        ? cov_exp["bias_gyro"].as<double>() : std::log10(robot.model.cov_bias_gyro_const);
    double ba_exp = cov_exp && cov_exp["bias_acc"]
        ? cov_exp["bias_acc"].as<double>() : std::log10(robot.model.cov_bias_acc_const);
    double pri_ori_exp = cov_exp && cov_exp["prior_orientation"]
        ? cov_exp["prior_orientation"].as<double>() : std::log10(robot.model.cov_prior_orientation_const);
    double pri_vel_exp = cov_exp && cov_exp["prior_velocity"]
        ? cov_exp["prior_velocity"].as<double>() : std::log10(robot.model.cov_prior_velocity_const);
    double pri_pos_exp = cov_exp && cov_exp["prior_position"]
        ? cov_exp["prior_position"].as<double>() : std::log10(robot.model.cov_prior_position_const);
    double pri_bg_exp = cov_exp && cov_exp["prior_bias_gyro"]
        ? cov_exp["prior_bias_gyro"].as<double>() : std::log10(robot.model.cov_prior_bias_gyro_const);
    double pri_ba_exp = cov_exp && cov_exp["prior_bias_acc"]
        ? cov_exp["prior_bias_acc"].as<double>() : std::log10(robot.model.cov_prior_bias_acc_const);
    
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
        // Load saved results configuration
        save_results = config["textfile"]["save_results"].as<bool>();
        saved_results_folder = config["textfile"]["saved_results_folder"].as<std::string>();
        if (csv_mode) {
            std::string file_name = config["textfile"]["file_name"].as<std::string>();
            LoadCSVData(file_name);
            start_index = config["textfile"]["start_count"].as<int>();
            time_length = config["textfile"]["end_count"].as<int>();
            
            // Cap end_count at actual CSV data size (accounting for starting point)
            int csv_data_size = static_cast<int>(CSVData.timestamp.size());

            
            if (start_index < 0 || start_index >= csv_data_size) {
                throw std::out_of_range("start_count must select an existing CSV row");
            }
            if (time_length < start_index) {
                throw std::invalid_argument("end_count must be at least start_count");
            }
            if (time_length >= csv_data_size) {
                std::cout << "Capping inclusive end_count at the final CSV row." << std::endl;
                time_length = csv_data_size - 1;
            }
            
            std::cout << "Initializing from CSV index " << start_index
                      << " and running updates from index " << (start_index + 1)
                      << " to " << time_length << std::endl;
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
            contact_nn = std::make_unique<ContactNN>(robot.model);
        }
    } else {
        std::cerr << "ERROR: Contact_estimation_NN section not found in config file!" << std::endl;
        std::cerr << "Please add 'Contact_estimation_NN: enabled: <true/false> file_path: <path_to_model>' to " << config_path_arg << std::endl;
        throw std::runtime_error("Contact model configuration not found in initialization YAML config");

    }
    
    // Call the existing Initialize function with loaded parameters
    Initialize(dt_val, cov_val_setting, initial_condition, sliding_window_size_arg);
}

void INEKFilter_manager::new_measurement(const Eigen::VectorXd &imu_arg, const Eigen::MatrixXd &motor_data_arg, const std::vector<bool> &contacts_arg)
{
    // ========================================================================
    // ROBOT-SPECIFIC ENCODER ASSIGNMENT
    // imu_arg: 6-element Eigen::VectorXd [omega_x, omega_y, omega_z, acc_x, acc_y, acc_z]
    // motor_data_arg: 2 x num_joints Eigen::MatrixXd
    //               Row 0: joint positions
    //               Row 1: joint velocities
    // ========================================================================
    
    // Extract IMU data
    new_measurement(imu_arg);
    
    // Extract encoder positions and velocities from matrix rows
    int num_joints = motor_data_arg.cols();
    std::vector<double> encoder_data(num_joints);
    std::vector<double> encoderdot_data(num_joints);
    
    for (int i = 0; i < num_joints; ++i) {
        encoder_data[i] = motor_data_arg(0, i);      // Row 0: positions
        encoderdot_data[i] = motor_data_arg(1, i);   // Row 1: velocities
    }
    
    // Map directly to Eigen vectors (no conversion needed)
    ENCODER[frame_count] = robot.model.map_indices_to_encoders(encoder_data, "encoder");
    ENCODERDOT[frame_count] = robot.model.map_indices_to_encoders(encoderdot_data, "encoderdot");
    
    CONTACT_t[frame_count].contact = contacts_arg;
    
    // Pinnocchio FK and Jacobians - compute forward kinematics and joint Jacobians as soon as you get new measurement
    robot.model.Forward_Kinematics(ENCODER[frame_count], ENCODERDOT[frame_count]);
    robot.model.compute_joint_jacobians();

}

void INEKFilter_manager::new_measurement(const Eigen::VectorXd &imu_arg)
{
    if (imu_arg.size() != 6 || !imu_arg.allFinite()) {
        throw std::invalid_argument("Expected six finite IMU values");
    }
    IMU_Gyro[frame_count] = imu_arg.head<3>();
    IMU_Acc[frame_count] = imu_arg.tail<3>();
    if (!imu_history_initialized) {
        // Live startup has no earlier sample; seed history from the first IMU.
        IMU_Gyro[frame_count - 1] = IMU_Gyro[frame_count];
        IMU_Acc[frame_count - 1] = IMU_Acc[frame_count];
        imu_history_initialized = true;
    }
}

void INEKFilter_manager::Propagate_and_Correct()

{


    imu_measurement << IMU_Gyro[frame_count - 1], IMU_Acc[frame_count - 1];
    filter.Propagate(imu_measurement, dt, Variable_Contact_Cov(frame_count - 1));


    // // Build contact constraint list from slip-filtered hard contacts
    std::vector<pair<int,bool>> contactConstraints;
    for (int i=0; i<robot.model.num_legs; i++) {
        contactConstraints.emplace_back(i, CONTACT_t[frame_count].contact[i]);
    }

    filter.setContacts(contactConstraints);


    int id;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double,6,6> covariance = Eigen::Matrix<double,6,6>::Zero();
    inekf::vectorKinematics measured_kinematics;
    

    // Build measured kinematics from batch results
    for (int k=0; k<robot.model.num_legs; k++) {
        id = k;
        pose.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
        pose.block<3,1>(0,3) = robot.model.get_foot_position(k);
        
        // IMU-relative position Jacobian in IMU axes, with active encoder columns only.
        const Eigen::MatrixXd jacobian = robot.model.get_foot_jacobian(k);
        covariance.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * cov_enc_const;
        covariance.block<3,3>(3,3) =
            cov_enc_const * jacobian * jacobian.transpose();

        // Contact-NN foot velocity is separate from the future NN base-velocity measurement.
        if (contact_nn_enabled && dv_norm_nn.size() != 0 && kinematics_mode == "vkm") {
            const Eigen::Vector3d nn_variance = NN_foot_velocity_variance_world.segment<3>(3 * k);
            if (nn_variance.allFinite() && (nn_variance.array() > 0.0).all()) {
                covariance.block<3,3>(0,0) = nn_variance.asDiagonal();
            }
            inekf::Kinematics frame(id, pose, covariance, robot.model.get_foot_velocity(k), IMU_Gyro[frame_count]);
            frame.world_foot_velocity = NN_foot_velocity_world.segment<3>(3 * k);
            frame.use_world_foot_velocity = true;
            measured_kinematics.push_back(frame);
        } else {
            inekf::Kinematics frame(id, pose, covariance, robot.model.get_foot_velocity(k), IMU_Gyro[frame_count]);
            measured_kinematics.push_back(frame);
        }
    }

    filter.CorrectKinematics(kinematics_mode, measured_kinematics, true);
    auto state = filter.getState();
    store_state_history(state);
    

        // Compute d_v from kinematics (full 3D velocity)
        for (int k=0; k<robot.model.num_legs; k++){
            CONTACT_t[frame_count].d_v.block(3*k,0,3,1) = state.getVelocity()
                    + state.getRotation()*robot.model.get_foot_velocity(k)
                    + state.getRotation()*BasicFunctions_Estimator::Hat_so3(IMU_Gyro[frame_count] - state.getGyroscopeBias())*robot.model.get_foot_position(k);
            
            CONTACT_t[frame_count].dv_norm(k) = CONTACT_t[frame_count].d_v.block(3*k,0,3,1).norm();
        }
        // Apply this step's NN foot velocities and norms after the kinematic defaults.
        if (contact_nn_enabled && dv_norm_nn.size() != 0) {
            CONTACT_t[frame_count].d_v = NN_foot_velocity_world;
            CONTACT_t[frame_count].dv_norm = dv_norm_nn;
        }
        dv_norm_nn.resize(0);
        
        for (int k=0; k<robot.model.num_legs; k++){
            CONTACT_t[frame_count].slip[k] = false;
            
            if (slip_rejection_mode == true && CONTACT_t[frame_count].contact[k] == true &&
                CONTACT_t[frame_count].dv_norm(k) > slip_threshold){
                CONTACT_t[frame_count].slip[k] = true;
            }
            CONTACT_t[frame_count].hard_contact[k]  = CONTACT_t[frame_count].contact[k] - CONTACT_t[frame_count].slip[k];
    }
    
    

}

void INEKFilter_manager::send_states(RobotManager::ManagerState &state_arg) {
    auto filter_state = filter.getState();
    const Eigen::Vector3d w_IMU_world = filter_state.getRotation()
        * (IMU_Gyro[frame_count] - filter_state.getGyroscopeBias());
    robot.model.transform_state_to_frame(
        robot.model.state_frame_id,
        filter_state.getRotation(), filter_state.getPosition(),
        filter_state.getVelocity(), w_IMU_world,
        state_arg.Rotation, state_arg.Position, state_arg.Velocity, state_arg.Angular_Velocity);
    state_arg.Covariance = filter_state.getP();

    // Populate remaining robot.state fields (sensor-specific and contact data)
    state_arg.Bias_Gyro = filter_state.getGyroscopeBias();
    state_arg.Bias_Acc = filter_state.getAccelerometerBias();
    state_arg.Contact = CONTACT_t[frame_count].contact;
    state_arg.Hard_Contact = CONTACT_t[frame_count].hard_contact;
    state_arg.Slip = CONTACT_t[frame_count].slip;
    state_arg.d_v = CONTACT_t[frame_count].d_v;
    state_arg.dv_norm = CONTACT_t[frame_count].dv_norm;


}

void INEKFilter_manager::store_state_history(inekf::RobotState& state_arg) {
    Rotation_s[frame_count] = state_arg.getRotation();
    Position_s[frame_count] = state_arg.getPosition();
    Velocity_s[frame_count] = state_arg.getVelocity();
    Bias_Gyro_s[frame_count] = state_arg.getGyroscopeBias();
    Bias_Acc_s[frame_count] = state_arg.getAccelerometerBias();
}

void INEKFilter_manager::sliding_window() {

// Shift history toward previous index and reset the last slot for fresh measurements.
    if (sliding_window_size < 2) return;

    for (int i = 0; i + 1 < sliding_window_size; ++i) {
        Position_s[i] = Position_s[i + 1];
        Rotation_s[i] = Rotation_s[i + 1];
        Velocity_s[i] = Velocity_s[i + 1];
        Bias_Gyro_s[i] = Bias_Gyro_s[i + 1];
        Bias_Acc_s[i] = Bias_Acc_s[i + 1];
        CONTACT_t[i] = CONTACT_t[i + 1];
        IMU_Gyro[i] = IMU_Gyro[i + 1];
        IMU_Acc[i] = IMU_Acc[i + 1];
        ENCODER[i] = ENCODER[i + 1];
        ENCODERDOT[i] = ENCODERDOT[i + 1];
        MOTOR_TORQUES[i] = MOTOR_TORQUES[i + 1];
    }

    frame_count = sliding_window_size - 1;
    Position_s[frame_count].setZero();
    Rotation_s[frame_count].setIdentity();
    Velocity_s[frame_count].setZero();
    Bias_Gyro_s[frame_count].setZero();
    Bias_Acc_s[frame_count].setZero();
    CONTACT_t[frame_count].reset(robot.model.num_legs, std::numeric_limits<double>::infinity());
    IMU_Gyro[frame_count].setZero();
    IMU_Acc[frame_count].setZero();
    ENCODER[frame_count].setZero();
    ENCODERDOT[frame_count].setZero();
    MOTOR_TORQUES[frame_count].setZero();
}

void INEKFilter_manager::Onestep(const Eigen::VectorXd& fresh_imu_arg) { // Look later and streamline
    new_measurement(fresh_imu_arg);
    imu_measurement << IMU_Gyro[frame_count - 1], IMU_Acc[frame_count - 1];
    filter.Propagate(imu_measurement, dt, Variable_Contact_Cov(frame_count - 1));
    auto state = filter.getState();
    store_state_history(state);
    CONTACT_t[frame_count].reset(robot.model.num_legs, std::numeric_limits<double>::quiet_NaN());
    CONTACT_t[frame_count].contact = CONTACT_t[frame_count - 1].contact;
    dv_norm_nn.resize(0);
    send_states(robot.state);
    sliding_window();
}

void INEKFilter_manager::Onestep(const Eigen::VectorXd& fresh_imu_arg, const Eigen::VectorXd& nn_velocity_arg) {
    if (nn_velocity_arg.size() != 6 || !nn_velocity_arg.allFinite() ||
        (nn_velocity_arg.tail<3>().array() <= 0.0).any()) {
        throw std::invalid_argument("Expected body velocity and three finite positive variances");
    }
    new_measurement(fresh_imu_arg);
    imu_measurement << IMU_Gyro[frame_count - 1], IMU_Acc[frame_count - 1];
    filter.Propagate(imu_measurement, dt, Variable_Contact_Cov(frame_count - 1));
    filter.CorrectBodyVelocity(nn_velocity_arg.head<3>(), nn_velocity_arg.tail<3>());
    auto state = filter.getState();
    store_state_history(state);
    CONTACT_t[frame_count].reset(robot.model.num_legs, std::numeric_limits<double>::quiet_NaN());
    CONTACT_t[frame_count].contact = CONTACT_t[frame_count - 1].contact;
    dv_norm_nn.resize(0);
    send_states(robot.state);
    sliding_window();
}

void INEKFilter_manager::Onestep(const Eigen::VectorXd& fresh_imu_arg, const std::vector<bool>&,
                        const Eigen::MatrixXd&, const Eigen::VectorXd&) {
    throw std::logic_error("TODO: implement contact kinematics plus separate velocity NN correction");
}

void INEKFilter_manager::Onestep(const Eigen::VectorXd &fresh_imu_arg, const std::vector<bool> &contacts_arg, const Eigen::MatrixXd &motor_data_arg) {
    if (fresh_imu_arg.size() != 6 || !fresh_imu_arg.allFinite() ||
        motor_data_arg.rows() != 2 ||
        !motor_data_arg.allFinite() || contacts_arg.size() != static_cast<std::size_t>(robot.model.num_legs)) {
        throw std::invalid_argument("Expected IMU, one contact per leg, and finite motor data [q; qd]");
    }

    // Current IMU remains in history for correction, residuals, and output.
    new_measurement(fresh_imu_arg, motor_data_arg, contacts_arg);
    Propagate_and_Correct();
    send_states(robot.state);

    if (!filter.getState().getVelocity().allFinite() || filter.getState().getVelocity().norm() > 10) {
        std::cerr << "Estimator diverged" << std::endl;
    }

    // Preserve the completed frame in history and clear the fresh slot.
    sliding_window();

}

void INEKFilter_manager::load_contact_model(const std::string& model_path_arg) {
    // Create ContactNN instance if not already created
    if (!contact_nn) {
        contact_nn = std::make_unique<ContactNN>(robot.model);
    }
    // Delegate to ContactNN
    contact_nn->load_contact_model(model_path_arg);
}

std::tuple<std::vector<bool>, Eigen::VectorXd> INEKFilter_manager::run_contact_inference(const Eigen::VectorXd &imu_arg, // Look at this implementation later, its very broken right now
        const Eigen::MatrixXd &motor_data_arg, const Eigen::VectorXd &motor_torques_arg,
        const Eigen::VectorXd &motor_pos_cmd_arg, const Eigen::VectorXd &cmd_velocity_arg) {

    if (imu_arg.size() != 6 || !imu_arg.allFinite() || motor_data_arg.rows() != 2 ||
        !motor_data_arg.allFinite()) {
        throw std::invalid_argument("Expected finite IMU and motor data [q; qd] for contact inference");
    }

    // Prepare NN inputs independently; Onestep stores the measurement with inferred contacts.
    std::vector<double> positions(motor_data_arg.cols()), velocities(motor_data_arg.cols());
    for (int joint = 0; joint < motor_data_arg.cols(); ++joint) {
        positions[joint] = motor_data_arg(0, joint);
        velocities[joint] = motor_data_arg(1, joint);
    }
    Eigen::VectorXd encoder = robot.model.map_indices_to_encoders(positions, "encoder");
    Eigen::VectorXd encoderdot = robot.model.map_indices_to_encoders(velocities, "encoderdot");
    robot.model.Forward_Kinematics(encoder, encoderdot);

    if (!contact_nn || !contact_nn->is_enabled()) throw std::logic_error("Contact NN is not loaded");

    // Store raw motor torques for NN feature extraction
    MOTOR_TORQUES[frame_count] = motor_torques_arg;

    // Extract NN features from current state (includes q, qd, acc, omega, p, v, tau)
    Eigen::VectorXd features = contact_nn->extract_nn_features(
        encoder,
        encoderdot,
        imu_arg.tail<3>(),
        imu_arg.head<3>(),
        MOTOR_TORQUES[frame_count],
        motor_pos_cmd_arg,
        cmd_velocity_arg
    );

    // Update sliding window buffer with new features
    contact_nn->update_nn_feature_buffer(features);

    // Predict contacts, world-frame foot velocities, and their diagonal variances.
    auto [predicted_contacts, predicted_foot_velocities, predicted_foot_velocity_variances] = contact_nn->nn_predict_contact();

    CONTACT_t[frame_count].contact = predicted_contacts;

    NN_foot_velocity_world = predicted_foot_velocities;
    NN_foot_velocity_variance_world = predicted_foot_velocity_variances;
    dv_norm_nn.resize(robot.model.num_legs);
    for (int leg = 0; leg < robot.model.num_legs; ++leg) {
        dv_norm_nn(leg) = predicted_foot_velocities.segment<3>(3 * leg).norm();
    }
    return {CONTACT_t[frame_count].contact, dv_norm_nn};
}

std::vector<bool> INEKFilter_manager::run_contact_detection(const Eigen::VectorXd &motor_torques_arg,
        const Eigen::VectorXd &joint_velocities_arg) {
    if (csv_mode) throw std::logic_error("Model-based contact detection requires CSV mode to be disabled");
    dv_norm_nn.resize(0);
    // Detection precedes Onestep, so use the previous completed filter residual.
    CONTACT_t[frame_count].contact = ContactDetection::get_contacts(
        motor_torques_arg, joint_velocities_arg, CONTACT_t[frame_count > 0 ? frame_count - 1 : 0].d_v, robot.model.Leg_indices_map);
    return CONTACT_t[frame_count].contact;
}

Eigen::VectorXd INEKFilter_manager::ReadCSVIMU(int row_arg) const {
    Eigen::VectorXd imu_i(6);
    imu_i << CSVData.gyro_body_x.at(row_arg), CSVData.gyro_body_y.at(row_arg), CSVData.gyro_body_z.at(row_arg),
             CSVData.acc_body_x.at(row_arg), CSVData.acc_body_y.at(row_arg), CSVData.acc_body_z.at(row_arg);
    if (robot.model.robot_name == "G1") {
        Eigen::Vector4d quaternion;
        quaternion << CSVData.quat_w.at(row_arg), CSVData.quat_i.at(row_arg),
                      CSVData.quat_j.at(row_arg), CSVData.quat_k.at(row_arg);
        const Eigen::Matrix3d rotation = BasicFunctions_Estimator::Quaternion_to_Rotation_Matrix(quaternion);
        imu_i.tail<3>() -= rotation.transpose() * gravity;
    }
    return imu_i;
}

std::tuple<Eigen::VectorXd, std::vector<bool>, Eigen::MatrixXd>
INEKFilter_manager::ReadCSVInputs(int row_arg) const {
    Eigen::VectorXd imu_i = ReadCSVIMU(row_arg);
    Eigen::MatrixXd motor_data_i;
    const auto& q = CSVData.encoders.at(row_arg);
    const auto& qd = CSVData.encoderdots.at(row_arg);
    if (q.size() != qd.size()) throw std::invalid_argument("CSV q and qd lengths differ");
    motor_data_i.resize(2, q.size());
    for (std::size_t joint = 0; joint < q.size(); ++joint) {
        motor_data_i(0, joint) = q[joint];
        motor_data_i(1, joint) = qd[joint];
    }
    return {imu_i, CSVData.contact_state.at(row_arg), motor_data_i};
}

void INEKFilter_manager::RunCSVStep() {
    if (!csv_mode) throw std::logic_error("RunCSVStep requires loaded CSV data");
    const int j = time_count + start_index + 1;
    if (j > time_length || j >= static_cast<int>(CSVData.timestamp.size())) {
        throw std::out_of_range("No CSV rows remain within inclusive end_count");
    }
    auto [imu, contacts, motor_data] = ReadCSVInputs(j);
    Onestep(imu, contacts, motor_data);
    // The completed sample is now in the penultimate slot; the logger reads frame_count.
    frame_count = sliding_window_size - 2;
    GroundTruthContact = CSVData.contact_state[j]; // For evaluation purposes, store ground truth contact from CSV
    
    // // Print contact and d_v info for debugging
    // std::cout << "Contacts: [" << CONTACT_t[frame_count].contact[0] << ", " << CONTACT_t[frame_count].contact[1] << "]";
    // std::cout << " GT: [" << GroundTruthContact[0] << ", " << GroundTruthContact[1] << "]" << std::endl;
    // std::cout << "d_v norms: [" << CONTACT_t[frame_count].d_v.block(0,0,3,1).norm() << ", " << CONTACT_t[frame_count].d_v.block(3,0,3,1).norm() << "]" << std::endl;

    // Compare detected contact with ground truth and update counters
    for (int k = 0; k < robot.model.num_legs; k++) {
        if (CONTACT_t[frame_count].contact[k] == GroundTruthContact[k]) {
            contact_match_count[k]++;
        } else {
            contact_mismatch_count[k]++;
        }
    }
    total_comparisons++;

    InEKF_SaveUtils::save_onestep_impl(*this, time_count + 1);
    frame_count = sliding_window_size - 1;
    ++time_count;
}

void INEKFilter_manager::getParameters(const std::string& urdf_path_arg, const std::string& robot_name_arg) {
    // Wrapper function that calls the utility
    extractParameters(urdf_path_arg, robot_name_arg);
}
