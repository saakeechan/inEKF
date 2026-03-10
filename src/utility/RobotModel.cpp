//
// Created by Saaketh 01/06/26
// Unified Robot Model Implementation with YAML Configuration Support
//

#include "RobotModel.hpp"
#include <iostream>
#include <fstream>
#include <stdexcept>

RobotModel::RobotModel() {
    // Initialize default values
    IMU2BD.setZero();
    
    // Initialize covariance constants with defaults
    cov_gyro_const = 1e-4;
    cov_acc_const = 1e-2;
    cov_slip_const = 1e-3;
    cov_contact_const = 1e-3;
    cov_enc_const = 1e-4;
    cov_bias_gyro_const = 1e-6;
    cov_bias_acc_const = 1e-6;
    cov_prior_orientation_const = 1e-3;
    cov_prior_velocity_const = 1e-3;
    cov_prior_position_const = 1e-3;
    cov_prior_bias_gyro_const = 1e-6;
    cov_prior_bias_acc_const = 1e-6;
}

RobotModel::~RobotModel() {
    // Cleanup handled automatically by smart pointers
}

bool RobotModel::load_config(const std::string& config_file) {
    try {
        std::cout << "Loading robot configuration from: " << config_file << std::endl;
        
        // Load YAML file
        config = YAML::LoadFile(config_file);
        
        // ====================================================================
        // ROBOT IDENTIFICATION
        // ====================================================================
        robot_name = config["robot_name"].as<std::string>();
        
        // ====================================================================
        // ROBOT MODEL CONFIGURATION
        // ====================================================================
        robot_model_path = config["model"]["model_path"].as<std::string>();
        
        // Read model format from YAML (required field)
        if (!config["model"]["format"]) {
            throw std::runtime_error("Model format must be specified in YAML: 'format: urdf' or 'format: mjcf'");
        }
        model_format = config["model"]["format"].as<std::string>();
        
        // Validate format
        if (model_format != "urdf" && model_format != "mjcf") {
            throw std::runtime_error("Invalid model format: " + model_format + ". Must be 'urdf' or 'mjcf'");
        }
        
        // ====================================================================
        // ROBOT STRUCTURE
        // ====================================================================
        num_legs = config["structure"]["num_legs"].as<int>();
        
        // Load contact frame IDs
        contact_point_frame_ids = config["structure"]["contact_frame_ids"].as<std::vector<int>>();
        
        std::cout << "Structure: " << num_legs << " legs, " << contact_point_frame_ids.size() << " contacts" << std::endl;
        // Note: encoder_size and encoderdot_size will be set after loading the robot model
        
        // ====================================================================
        // ENCODER MAPPING
        // ====================================================================
        if (config["Encoder_indices_map"]) {
            Encoder_indices_map = config["Encoder_indices_map"].as<std::vector<int>>();
            std::cout << "Loaded " << Encoder_indices_map.size() << " Encoder position mapping indices for pinnochio" << std::endl;
        }
        if (config["Encoderdot_indices_map"]) {
            Encoderdot_indices_map = config["Encoderdot_indices_map"].as<std::vector<int>>();
            std::cout << "Loaded " << Encoderdot_indices_map.size() << " Encoder velocity mapping indices for pinnochio" << std::endl;
        }
        
        // ====================================================================
        // TORQUE MAPPING (Optional, but must match num_legs if provided)
        // ====================================================================
        if (config["Torque_indices_map"]) {
            Torque_indices_map = config["Torque_indices_map"].as<std::vector<std::vector<int>>>();
            
            // Validate that the number of rows matches num_legs
            if (Torque_indices_map.size() != static_cast<size_t>(num_legs)) {
                throw std::runtime_error(
                    "Torque_indices_map has " + std::to_string(Torque_indices_map.size()) + 
                    " rows, but num_legs is " + std::to_string(num_legs) + 
                    ". Number of rows must equal number of legs."
                );
            }
            
            std::cout << "Loaded Torque_indices_map with " << Torque_indices_map.size() << " legs:" << std::endl;
            for (size_t i = 0; i < Torque_indices_map.size(); ++i) {
                std::cout << "  Leg " << i << ": " << Torque_indices_map[i].size() << " torque indices" << std::endl;
            }
        } else {
            std::cout << "Torque_indices_map not provided (optional)" << std::endl;
        }
        
        // ====================================================================
        // IMU2BD OFFSET
        // =====================================================================
        // IMU offset
        if (config["imu_to_body"]) {
            auto imu_offset = config["imu_to_body"].as<std::vector<double>>();
            IMU2BD << imu_offset[0], imu_offset[1], imu_offset[2];
        }
        
        // ====================================================================
        // LOAD COVARIANCE PARAMETERS
        // ====================================================================
        load_covariance_params();
        initialize_covariances();
        
        // ====================================================================
        // LOAD ROBOT MODEL
        // ====================================================================
        load_robot_model(robot_model_path, contact_point_frame_ids);
        
        std::cout << "Robot configuration loaded successfully!" << std::endl;
        return true;
        
    } catch (const YAML::Exception& e) {
        std::cerr << "YAML parsing error: " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Error loading config: " << e.what() << std::endl;
        return false;
    }
}

void RobotModel::load_covariance_params() {
    if (!config["covariance"]) {
        std::cout << "No covariance parameters in config, using defaults" << std::endl;
        return;
    }
    
    auto cov = config["covariance"]; // Test with partial covariance specification
    
    cov_gyro_const = cov["gyro"].as<double>(cov_gyro_const);
    cov_acc_const = cov["accelerometer"].as<double>(cov_acc_const);
    cov_enc_const = cov["encoder"].as<double>(cov_enc_const);
    cov_contact_const = cov["contact"].as<double>(cov_contact_const);
    cov_slip_const = cov["slip"].as<double>(cov_slip_const);
    cov_bias_gyro_const = cov["bias_gyro"].as<double>(cov_bias_gyro_const);
    cov_bias_acc_const = cov["bias_accelerometer"].as<double>(cov_bias_acc_const);
    
    cov_prior_orientation_const = cov["prior_orientation"].as<double>(cov_prior_orientation_const);
    cov_prior_velocity_const = cov["prior_velocity"].as<double>(cov_prior_velocity_const);
    cov_prior_position_const = cov["prior_position"].as<double>(cov_prior_position_const);
    cov_prior_bias_gyro_const = cov["prior_bias_gyro"].as<double>(cov_prior_bias_gyro_const);
    cov_prior_bias_acc_const = cov["prior_bias_accelerometer"].as<double>(cov_prior_bias_acc_const);
    
    std::cout << "Loaded covariance parameters from config" << std::endl;
}

void RobotModel::initialize_covariances() {
    // Initialize covariance matrices as diagonal matrices
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
    SQRT_INFO_Covariance_Prior_Orientation = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_prior_orientation_const);
    SQRT_INFO_Covariance_Prior_Velocity = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_prior_velocity_const);
    SQRT_INFO_Covariance_Prior_Position = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_prior_position_const);
    SQRT_INFO_Covariance_Prior_Bias_Gyro = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_prior_bias_gyro_const);
    SQRT_INFO_Covariance_Prior_Bias_Acc = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_prior_bias_acc_const);
}


void RobotModel::load_robot_model(const std::string& xml_path, const std::vector<int>& contact_frame_ids) {
    try {
        // Load model based on format specified in config
        if (model_format == "mjcf") {
            pinocchio::mjcf::buildModel(xml_path, pin_model);
        } else if (model_format == "urdf") {
            pinocchio::urdf::buildModel(xml_path, pin_model);
        } else {
            throw std::runtime_error("Unsupported model format: " + model_format + " Must be 'urdf' or 'mjcf'");
        }
        
        pin_data = pinocchio::Data(pin_model);
        
        std::cout << "Successfully loaded robot model from: " << xml_path << std::endl;
        std::cout << "Number of joints: " << pin_model.njoints << std::endl;
        std::cout << "Number of frames: " << pin_model.nframes << std::endl;
        
        // Set encoder dimensions from the loaded model
        encoder_size = pin_model.nq;
        encoderdot_size = pin_model.nv;
        std::cout << "Set encoder_size=" << encoder_size << ", encoderdot_size=" << encoderdot_size << " from model" << std::endl;
        
        // Setup contact frame IDs - convert int to FrameIndex
        foot_frame_ids.assign(contact_frame_ids.begin(), contact_frame_ids.end());
        
        if (!foot_frame_ids.empty()) { // Just a bigass unnecessary print statement...check later and remove
            std::cout << "Loaded " << foot_frame_ids.size() << " contact frames: [";
            for (size_t i = 0; i < foot_frame_ids.size(); i++) {
                std::cout << foot_frame_ids[i];
                if (i < foot_frame_ids.size() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        } else {
            std::cerr << "Warning: No contact frame IDs provided" << std::endl;
            std::exit(EXIT_FAILURE); 

        }
        
    } catch (const std::exception& e) {
        std::cerr << "Failed to load robot model: " << e.what() << std::endl;
        std::cerr << "Model path: " << xml_path << std::endl;
        throw;
    }
}

void RobotModel::SetCovariances(Eigen::Matrix<double,12,1> cov_val_setting) {

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
    SQRT_INFO_Covariance_Prior_Orientation = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_prior_orientation_const);
    SQRT_INFO_Covariance_Prior_Velocity = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_prior_velocity_const);
    SQRT_INFO_Covariance_Prior_Position = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_prior_position_const);
    SQRT_INFO_Covariance_Prior_Bias_Gyro = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_prior_bias_gyro_const);
    SQRT_INFO_Covariance_Prior_Bias_Acc = Eigen::Matrix<double, 3, 3>::Identity() / sqrt(cov_prior_bias_acc_const);
}

///////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////     Kinematics Functions  ///////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

void RobotModel::Forward_Kinematics(const Eigen::VectorXd &q_leg, const Eigen::VectorXd &q_dot_leg) const {
    if (foot_frame_ids.empty()) {
        std::cerr << "Error: Pinocchio not initialized properly" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    
    // Compute forward kinematics - updates all joint and frame placements
    pinocchio::forwardKinematics(pin_model, const_cast<pinocchio::Data&>(pin_data), q_leg, q_dot_leg);
    
    // Update all frame placements (foot frames, etc.)
    pinocchio::updateFramePlacements(pin_model, const_cast<pinocchio::Data&>(pin_data));
}

Eigen::Vector3d RobotModel::get_foot_position(int i_leg) const {
    if (foot_frame_ids.empty()) {
        std::cerr << "Error: Pinocchio not initialized properly" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    
    if (i_leg < 1 || i_leg > static_cast<int>(foot_frame_ids.size())) {
        std::cerr << "Error: Invalid leg index " << i_leg << ". Must be between 1 and " 
                  << foot_frame_ids.size() << std::endl;
        std::exit(EXIT_FAILURE);
    }
    
    // Get foot position from the already-computed forward kinematics
    const pinocchio::SE3& foot_placement = pin_data.oMf[foot_frame_ids[i_leg - 1]];
    Eigen::Vector3d foot_position = foot_placement.translation();
    
    // return foot_position + IMU2BD;
    return foot_position; // We are using an IMU-shifted model, so no need to add IMU2BD offset here
}

Eigen::MatrixXd RobotModel::get_foot_jacobian(int i_leg) const {
    if (foot_frame_ids.empty()) {
        std::cerr << "Error: Pinocchio not initialized properly" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    
    if (i_leg < 1 || i_leg > static_cast<int>(foot_frame_ids.size())) {
        std::cerr << "Error: Invalid leg index " << i_leg << ". Must be between 1 and " 
                  << foot_frame_ids.size() << std::endl;
        std::exit(EXIT_FAILURE);
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

Eigen::Vector3d RobotModel::get_foot_velocity(int i_leg) const {
    if (foot_frame_ids.empty()) {
        std::cerr << "Error: Pinocchio not initialized properly" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    
    if (i_leg < 1 || i_leg > static_cast<int>(foot_frame_ids.size())) {
        std::cerr << "Error: Invalid leg index " << i_leg << ". Must be between 1 and " 
                  << foot_frame_ids.size() << std::endl;
        std::exit(EXIT_FAILURE);
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

void RobotModel::compute_joint_jacobians() const {
    
    // Compute all joint Jacobians using the configuration already in pin_data
    // This prepares the data structure for efficient frame Jacobian queries
    pinocchio::computeJointJacobians(pin_model, const_cast<pinocchio::Data&>(pin_data));
}

void RobotModel::map_indices_to_encoders(const std::vector<double>& motor_data, Eigen::VectorXd& output_vector, const std::string& encoder_type) const {

    // Validate encoder_type parameter
    if (encoder_type != "encoder" && encoder_type != "encoderdot") {
        std::cerr << "Error: Invalid encoder_type '" << encoder_type << "'. Must be 'encoder' or 'encoderdot'" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    
    // Select appropriate indices based on encoder_type
    const std::vector<int>& indices = (encoder_type == "encoder") ? Encoder_indices_map : Encoderdot_indices_map;
    const std::string type_name = (encoder_type == "encoder") ? "encoder position" : "encoder velocity";
    
    // Check if indices are configured
    if (indices.empty()) {
        std::cerr << "Error: " << type_name << " indices not configured in YAML file" << std::endl;
        std::exit(EXIT_FAILURE);
    }
    
    // Check if motor_data size exactly matches the number of indices
    if (motor_data.size() != indices.size()) {
        std::cerr << "FATAL ERROR: TextFile data size mismatch for " << type_name << "." << std::endl;
        std::cerr << "  Expected exactly " << indices.size() << " values, got " << motor_data.size() << std::endl;
        std::cerr << "  This is a configuration error - check your TextFileDataStruct and YAML indices." << std::endl;
        std::cerr << "  Debug: motor_data contents: ";
        for (size_t i = 0; i < motor_data.size(); i++) {
            std::cerr << motor_data[i] << " ";
        }
        std::cerr << std::endl;
        std::exit(EXIT_FAILURE);
    }
    
    // Initialize output_vector to zero (all non-mapped indices will remain zero)
    output_vector.setZero();
    
    // Map motor_data to output_vector using the index mapping
    // indices[i] tells us WHERE to put the data in output_vector
    // We take data from position i in motor_data
    for (size_t i = 0; i < indices.size(); ++i) {
        int output_idx = indices[i];
        
        // Validate output index bounds
        if (output_idx < 0 || output_idx >= output_vector.size()) {
            std::cerr << "FATAL ERROR: Invalid " << encoder_type << " mapping index " << output_idx 
                      << " at position " << i << " for output vector of size " << output_vector.size() << std::endl;
            std::exit(EXIT_FAILURE);
        }
        
        // Map: motor_data[i] -> output_vector[indices[i]]
        output_vector(output_idx) = motor_data[i];
    }
}

Eigen::MatrixXd RobotModel::extract_active_encoder_jacobian(const Eigen::MatrixXd& full_jacobian) const {
    
    // Check if encoderdot indices are configured
    if (Encoderdot_indices_map.empty()) {
        std::cerr << "Warning: No encoderdot indices configured. Returning full Jacobian." << std::endl;
        return full_jacobian;
    }
    
    // Create output matrix with only columns specified in yaml config
    // Number of rows = same as input (typically 3 for position)
    // Number of cols = number of active encoders specified in yaml
    Eigen::MatrixXd active_jacobian(full_jacobian.rows(), Encoderdot_indices_map.size());
    
    // Extract only the columns corresponding to encoderdot_indices from yaml
    for (size_t i = 0; i < Encoderdot_indices_map.size(); ++i) {
        int col_idx = Encoderdot_indices_map[i];
        
        // Validate column index bounds
        if (col_idx < 0 || col_idx >= full_jacobian.cols()) {
            std::cerr << "Error: Encoderdot index " << col_idx << " out of bounds for Jacobian with " 
                      << full_jacobian.cols() << " columns" << std::endl;
            // Set to zero column if out of bounds
            active_jacobian.col(i).setZero();
            continue;
        }
        
        active_jacobian.col(i) = full_jacobian.col(col_idx);
    }
    
    return active_jacobian;
}

