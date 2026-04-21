/**
 * @file getFootPositionVelocityFK.cpp
 * @brief Forward kinematics calculator for foot position and velocity from CSV data
 * 
 * This file reads G1 robot CSV data, loads the robot model, and computes
 * foot positions and velocities using forward kinematics.
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "csvParserG1_FootPosVel.hpp"
#include "../../src/utility/RobotModel.hpp"

using std::cout;
using std::endl;

int main(int argc, char* argv[]) {
    
    // Check for command-line arguments
    std::string csv_filename;
    if (argc > 1) {
        csv_filename = argv[1];
    } else {
        // Default CSV file path
        csv_filename = "/home/saakee/Research/deep-contact-estimator/Data/CSVFiles/robotstate_0_env4.csv";
        cout << "No CSV file specified. Using default: " << csv_filename << endl;
    }
    
    // ========================================================================
    // STEP 1: Parse G1 CSV data
    // ========================================================================
    cout << "Loading CSV data from: " << csv_filename << endl;
    G1DataStruct g1_data = ParseG1CSV(csv_filename);
    
    // Verify data was loaded
    if (g1_data.timestamp.empty()) {
        std::cerr << "Error: Failed to load CSV data or CSV file is empty!" << endl;
        return -1;
    }
    
    cout << "Successfully loaded " << g1_data.timestamp.size() << " timesteps of data" << endl;
    
    // ========================================================================
    // STEP 2: Load robot model from G1 config file
    // ========================================================================
    std::string config_file = "../G1_config_FootPosVel.yaml";
    cout << "\nLoading robot model from config: " << config_file << endl;
    
    RobotModel robot;
    if (!robot.load_config(config_file)) {
        std::cerr << "Error: Failed to load robot configuration!" << endl;
        return -1;
    }
    
    cout << "Robot model successfully loaded and ready!" << endl;
    cout << "Robot: " << robot.robot_name << endl;
    cout << "Number of legs: " << robot.num_legs << endl;
    
    // ========================================================================
    // STEP 3: Compute forward kinematics for all timesteps
    // ========================================================================
    cout << "\n=== Computing forward kinematics ===" << endl;
    
    size_t num_timesteps = g1_data.timestamp.size();
    cout << "Processing " << num_timesteps << " timesteps..." << endl;
    
    // Storage for computed foot positions and velocities
    std::vector<Eigen::Vector3d> left_foot_positions, right_foot_positions;
    std::vector<Eigen::Vector3d> left_foot_velocities, right_foot_velocities;
    
    for (size_t t = 0; t < num_timesteps; ++t) {
        // Build joint position vector (29 joints for G1)
        Eigen::VectorXd q(29);
        q << g1_data.joint_pos_left_hip_pitch[t],
             g1_data.joint_pos_left_hip_roll[t],
             g1_data.joint_pos_left_hip_yaw[t],
             g1_data.joint_pos_left_knee[t],
             g1_data.joint_pos_left_ankle_pitch[t],
             g1_data.joint_pos_left_ankle_roll[t],
             g1_data.joint_pos_right_hip_pitch[t],
             g1_data.joint_pos_right_hip_roll[t],
             g1_data.joint_pos_right_hip_yaw[t],
             g1_data.joint_pos_right_knee[t],
             g1_data.joint_pos_right_ankle_pitch[t],
             g1_data.joint_pos_right_ankle_roll[t],
             g1_data.joint_pos_waist_yaw[t],
             g1_data.joint_pos_waist_roll[t],
             g1_data.joint_pos_waist_pitch[t],
             g1_data.joint_pos_left_shoulder_pitch[t],
             g1_data.joint_pos_left_shoulder_roll[t],
             g1_data.joint_pos_left_shoulder_yaw[t],
             g1_data.joint_pos_left_elbow[t],
             g1_data.joint_pos_left_wrist_roll[t],
             g1_data.joint_pos_left_wrist_pitch[t],
             g1_data.joint_pos_left_wrist_yaw[t],
             g1_data.joint_pos_right_shoulder_pitch[t],
             g1_data.joint_pos_right_shoulder_roll[t],
             g1_data.joint_pos_right_shoulder_yaw[t],
             g1_data.joint_pos_right_elbow[t],
             g1_data.joint_pos_right_wrist_roll[t],
             g1_data.joint_pos_right_wrist_pitch[t],
             g1_data.joint_pos_right_wrist_yaw[t];
        
        // Build joint velocity vector
        Eigen::VectorXd q_dot(29);
        q_dot << g1_data.joint_vel_left_hip_pitch[t],
                 g1_data.joint_vel_left_hip_roll[t],
                 g1_data.joint_vel_left_hip_yaw[t],
                 g1_data.joint_vel_left_knee[t],
                 g1_data.joint_vel_left_ankle_pitch[t],
                 g1_data.joint_vel_left_ankle_roll[t],
                 g1_data.joint_vel_right_hip_pitch[t],
                 g1_data.joint_vel_right_hip_roll[t],
                 g1_data.joint_vel_right_hip_yaw[t],
                 g1_data.joint_vel_right_knee[t],
                 g1_data.joint_vel_right_ankle_pitch[t],
                 g1_data.joint_vel_right_ankle_roll[t],
                 g1_data.joint_vel_waist_yaw[t],
                 g1_data.joint_vel_waist_roll[t],
                 g1_data.joint_vel_waist_pitch[t],
                 g1_data.joint_vel_left_shoulder_pitch[t],
                 g1_data.joint_vel_left_shoulder_roll[t],
                 g1_data.joint_vel_left_shoulder_yaw[t],
                 g1_data.joint_vel_left_elbow[t],
                 g1_data.joint_vel_left_wrist_roll[t],
                 g1_data.joint_vel_left_wrist_pitch[t],
                 g1_data.joint_vel_left_wrist_yaw[t],
                 g1_data.joint_vel_right_shoulder_pitch[t],
                 g1_data.joint_vel_right_shoulder_roll[t],
                 g1_data.joint_vel_right_shoulder_yaw[t],
                 g1_data.joint_vel_right_elbow[t],
                 g1_data.joint_vel_right_wrist_roll[t],
                 g1_data.joint_vel_right_wrist_pitch[t],
                 g1_data.joint_vel_right_wrist_yaw[t];
        
        // Compute forward kinematics
        robot.Forward_Kinematics(q, q_dot);
        
        // Get foot positions (1 = left foot, 2 = right foot)
        Eigen::Vector3d left_pos = robot.get_foot_position(1);
        Eigen::Vector3d right_pos = robot.get_foot_position(2);
        
        // Get foot velocities
        Eigen::Vector3d left_vel = robot.get_foot_velocity(1);
        Eigen::Vector3d right_vel = robot.get_foot_velocity(2);
        
        // Store results
        left_foot_positions.push_back(left_pos);
        right_foot_positions.push_back(right_pos);
        left_foot_velocities.push_back(left_vel);
        right_foot_velocities.push_back(right_vel);
        
        // Print progress every 100 timesteps
        if ((t + 1) % 100 == 0 || t == 0) {
            cout << "Processed timestep " << (t + 1) << "/" << num_timesteps << endl;
        }
    }
    
    cout << "\n=== Forward kinematics computation complete ===" << endl;
    cout << "Computed positions and velocities for " << left_foot_positions.size() << " timesteps" << endl;
    
    // Print sample results from first timestep
    cout << "\n=== Sample results from timestep 0 ===" << endl;
    cout << "Left foot position:  [" << left_foot_positions[0].transpose() << "]" << endl;
    cout << "Right foot position: [" << right_foot_positions[0].transpose() << "]" << endl;
    cout << "Left foot velocity:  [" << left_foot_velocities[0].transpose() << "]" << endl;
    cout << "Right foot velocity: [" << right_foot_velocities[0].transpose() << "]" << endl;

    // ========================================================================
    // STEP 4: Write results back to CSV with new columns
    // ========================================================================
    cout << "\n=== Writing results back to CSV ===" << endl;
    
    // Read original CSV file
    std::ifstream input_file(csv_filename);
    if (!input_file.is_open()) {
        std::cerr << "Error: Could not open CSV file for reading: " << csv_filename << endl;
        return -1;
    }
    
    std::vector<std::string> lines;
    std::string line;
    
    // Read header
    if (std::getline(input_file, line)) {
        // Append FK column headers
        line += ",fk_left_foot_pos_x,fk_left_foot_pos_y,fk_left_foot_pos_z";
        line += ",fk_right_foot_pos_x,fk_right_foot_pos_y,fk_right_foot_pos_z";
        line += ",fk_left_foot_vel_x,fk_left_foot_vel_y,fk_left_foot_vel_z";
        line += ",fk_right_foot_vel_x,fk_right_foot_vel_y,fk_right_foot_vel_z";
        lines.push_back(line);
    }
    
    // Read and append data rows
    size_t row_idx = 0;
    while (std::getline(input_file, line) && row_idx < num_timesteps) {
        std::ostringstream oss;
        oss << line;
        
        // Append FK data for this row
        oss << "," << left_foot_positions[row_idx](0)
            << "," << left_foot_positions[row_idx](1)
            << "," << left_foot_positions[row_idx](2);
        
        oss << "," << right_foot_positions[row_idx](0)
            << "," << right_foot_positions[row_idx](1)
            << "," << right_foot_positions[row_idx](2);
        
        oss << "," << left_foot_velocities[row_idx](0)
            << "," << left_foot_velocities[row_idx](1)
            << "," << left_foot_velocities[row_idx](2);
        
        oss << "," << right_foot_velocities[row_idx](0)
            << "," << right_foot_velocities[row_idx](1)
            << "," << right_foot_velocities[row_idx](2);
        
        lines.push_back(oss.str());
        row_idx++;
    }
    input_file.close();
    
    // Write back to the same file
    std::ofstream output_file(csv_filename);
    if (!output_file.is_open()) {
        std::cerr << "Error: Could not open CSV file for writing: " << csv_filename << endl;
        return -1;
    }
    
    for (const auto& output_line : lines) {
        output_file << output_line << "\n";
    }
    output_file.close();
    
    cout << "Successfully wrote " << row_idx << " rows with FK data to: " << csv_filename << endl;
    cout << "Added 12 new columns (FK positions and velocities for both feet)" << endl;
    
    return 0;
}