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
#include "csvParser.hpp"
#include "RobotModel.hpp"

using std::cout;
using std::endl;

int main() {
    
    // ========================================================================
    // File paths to process — add or remove paths as needed
    // ========================================================================
    const std::vector<std::string> csv_files = {
        "../contactData/robotstate_0_env0.csv",
        "../contactData/robotstate_0_env1.csv",
        "../contactData/robotstate_0_env2.csv",
        "../contactData/robotstate_0_env3.csv",
        "../contactData/robotstate_0_env4.csv",
        "../contactData/robotstate_0_env5.csv",
        "../contactData/robotstate_0_env6.csv",
        "../contactData/robotstate_0_env7.csv",
        "../contactData/robotstate_0_env8.csv",
        "../contactData/robotstate_0_env9.csv",
        "../contactData/robotstate_0_env10.csv",
        "../contactData/robotstate_0_env11.csv",
        "../contactData/robotstate_0_env12.csv",
        "../contactData/robotstate_0_env13.csv",
        "../contactData/robotstate_0_env14.csv",
        "../contactData/robotstate_0_env15.csv",
        "../contactData/robotstate_0_env16.csv",
        "../contactData/robotstate_0_env17.csv",
        "../contactData/robotstate_0_env18.csv",
        "../contactData/robotstate_0_env19.csv",
        "../contactData/robotstate_40000_env0.csv",
        "../contactData/robotstate_40000_env1.csv",
        "../contactData/robotstate_40000_env2.csv",
        "../contactData/robotstate_40000_env3.csv",
        "../contactData/robotstate_40000_env4.csv",
        "../contactData/robotstate_40000_env5.csv",
        "../contactData/robotstate_40000_env6.csv",
        "../contactData/robotstate_40000_env7.csv",
        "../contactData/robotstate_40000_env8.csv",
        "../contactData/robotstate_40000_env9.csv",
        "../contactData/robotstate_40000_env10.csv",
        "../contactData/robotstate_40000_env11.csv",
        "../contactData/robotstate_40000_env12.csv",
        "../contactData/robotstate_40000_env13.csv",
        "../contactData/robotstate_40000_env14.csv",
        "../contactData/robotstate_40000_env15.csv",
        "../contactData/robotstate_40000_env16.csv",
        "../contactData/robotstate_40000_env17.csv",
        "../contactData/robotstate_40000_env18.csv",
        "../contactData/robotstate_40000_env19.csv",
        "../contactData/robotstate_80000_env0.csv",
        "../contactData/robotstate_80000_env1.csv",
        "../contactData/robotstate_80000_env2.csv",
        "../contactData/robotstate_80000_env3.csv",
        "../contactData/robotstate_80000_env4.csv",
        "../contactData/robotstate_80000_env5.csv",
        "../contactData/robotstate_80000_env6.csv",
        "../contactData/robotstate_80000_env7.csv",
        "../contactData/robotstate_80000_env8.csv",
        "../contactData/robotstate_80000_env9.csv",
        "../contactData/robotstate_80000_env10.csv",
        "../contactData/robotstate_80000_env11.csv",
        "../contactData/robotstate_80000_env12.csv",
        "../contactData/robotstate_80000_env13.csv",
        "../contactData/robotstate_80000_env14.csv",
        "../contactData/robotstate_80000_env15.csv",
        "../contactData/robotstate_80000_env16.csv",
        "../contactData/robotstate_80000_env17.csv",
        "../contactData/robotstate_80000_env18.csv",
        "../contactData/robotstate_80000_env19.csv",
        "../contactData/robotstate_120000_env0.csv",
        "../contactData/robotstate_120000_env1.csv",
        "../contactData/robotstate_120000_env2.csv",
        "../contactData/robotstate_120000_env3.csv",
        "../contactData/robotstate_120000_env4.csv",
        "../contactData/robotstate_120000_env5.csv",
        "../contactData/robotstate_120000_env6.csv",
        "../contactData/robotstate_120000_env7.csv",
        "../contactData/robotstate_120000_env8.csv",
        "../contactData/robotstate_120000_env9.csv",
        "../contactData/robotstate_120000_env10.csv",
        "../contactData/robotstate_120000_env11.csv",
        "../contactData/robotstate_120000_env12.csv",
        "../contactData/robotstate_120000_env13.csv",
        "../contactData/robotstate_120000_env14.csv",
        "../contactData/robotstate_120000_env15.csv",
        "../contactData/robotstate_120000_env16.csv",
        "../contactData/robotstate_120000_env17.csv",
        "../contactData/robotstate_120000_env18.csv",
        "../contactData/robotstate_120000_env19.csv"
        // "../Data/synced.csv"
        // "../Data/synced copy.csv",
        // "../Data/synced copy 2.csv",
        // "../Data/synced copy 3.csv",
        // "../Data/synced copy 4.csv",
        // "../Data/synced copy 5.csv",
        // "../Data/synced copy 6.csv",
        // "../Data/synced copy 7.csv",
        // "../Data/synced copy 8.csv",
        // "../Data/synced copy 9.csv",
        // "../Data/synced copy 10.csv",
        // "../Data/synced copy 12.csv"
    };

    // ========================================================================
    // Load robot model once (shared across all files)
    // ========================================================================
    std::string config_file = "../config/G1_config.yaml";
    cout << "Loading robot model from config: " << config_file << endl;

    RobotModel robot;
    if (!robot.load_config(config_file)) {
        std::cerr << "Error: Failed to load robot configuration!" << endl;
        return -1;
    }

    cout << "Robot model successfully loaded and ready!" << endl;
    cout << "Robot: " << robot.robot_name << endl;
    cout << "Number of legs: " << robot.num_legs << endl;

    for (const auto& csv_filename : csv_files) {
    cout << "\n========================================" << endl;
    cout << "Processing: " << csv_filename << endl;
    cout << "========================================" << endl;

    // ========================================================================
    // STEP 1: Parse G1 CSV data
    // ========================================================================
    cout << "Loading CSV data from: " << csv_filename << endl;
    DataStruct data = ParseCSV(csv_filename);
    
    // Verify data was loaded
    if (data.timestamp.empty()) {
        std::cerr << "Error: Failed to load CSV data or CSV file is empty! Skipping." << endl;
        continue;
    }
    
    cout << "Successfully loaded " << data.timestamp.size() << " timesteps of data" << endl;
    
    // ========================================================================
    // STEP 3: Compute forward kinematics for all timesteps
    // ========================================================================
    cout << "\n=== Computing forward kinematics ===" << endl;
    
    size_t num_timesteps = data.timestamp.size();
    cout << "Processing " << num_timesteps << " timesteps..." << endl;
    
    // Storage for computed foot positions and velocities
    std::vector<Eigen::Vector3d> left_foot_positions, right_foot_positions;
    std::vector<Eigen::Vector3d> left_foot_velocities, right_foot_velocities;
    
    for (size_t t = 0; t < num_timesteps; ++t) {
        // Use pre-populated encoder arrays from csvParser (29 joints for G1)
        Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(data.encoders[t].data(), data.encoders[t].size());
        Eigen::VectorXd q_dot = Eigen::Map<const Eigen::VectorXd>(data.encoderdots[t].data(), data.encoderdots[t].size());
        
        // Debug: Print joint data for first timestep
        if (t == 0) {
            cout << "\n=== Joint data at timestep 0 ===" << endl;
            cout << "Total joints: " << q.size() << endl;
            cout << "Left leg positions (0-5):  " << q.segment(0, 6).transpose() << endl;
            cout << "Right leg positions (6-11): " << q.segment(6, 6).transpose() << endl;
            cout << "Left leg velocities (0-5):  " << q_dot.segment(0, 6).transpose() << endl;
            cout << "Right leg velocities (6-11): " << q_dot.segment(6, 6).transpose() << endl;
        }
        
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
        std::cerr << "Error: Could not open CSV file for reading: " << csv_filename << " Skipping." << endl;
        continue;
    }
    
    std::vector<std::string> lines;
    std::string line;

    // Helper: split a CSV line into fields
    auto split_csv = [](const std::string& s) {
        std::vector<std::string> fields;
        std::istringstream ss(s);
        std::string field;
        while (std::getline(ss, field, ',')) fields.push_back(field);
        return fields;
    };

    // FK column names and their corresponding data (in order)
    const std::vector<std::string> fk_col_names = {
        "fk_left_foot_pos_x",  "fk_left_foot_pos_y",  "fk_left_foot_pos_z",
        "fk_right_foot_pos_x", "fk_right_foot_pos_y", "fk_right_foot_pos_z",
        "fk_left_foot_vel_x",  "fk_left_foot_vel_y",  "fk_left_foot_vel_z",
        "fk_right_foot_vel_x", "fk_right_foot_vel_y", "fk_right_foot_vel_z"
    };

    // For each FK column: index in CSV (-1 = needs to be appended)
    std::vector<int> fk_col_indices(fk_col_names.size(), -1);
    std::vector<std::string> header_fields;

    if (std::getline(input_file, line)) {
        header_fields = split_csv(line);

        // Map existing column names to their indices
        for (size_t k = 0; k < fk_col_names.size(); ++k) {
            for (size_t i = 0; i < header_fields.size(); ++i) {
                if (header_fields[i] == fk_col_names[k]) {
                    fk_col_indices[k] = (int)i;
                    break;
                }
            }
            if (fk_col_indices[k] == -1) {
                // Column doesn't exist — append it and record its new index
                fk_col_indices[k] = (int)header_fields.size();
                header_fields.push_back(fk_col_names[k]);
                cout << "Appending new column: " << fk_col_names[k] << endl;
            } else {
                cout << "Overwriting existing column: " << fk_col_names[k] << " (index " << fk_col_indices[k] << ")" << endl;
            }
        }

        // Rebuild header line
        std::ostringstream hdr;
        for (size_t i = 0; i < header_fields.size(); ++i) {
            if (i > 0) hdr << ",";
            hdr << header_fields[i];
        }
        lines.push_back(hdr.str());
    }

    // FK values accessor: maps column index k -> value for timestep idx
    auto fk_value = [&](size_t k, size_t idx) -> double {
        switch (k) {
            case 0:  return left_foot_positions[idx](0);
            case 1:  return left_foot_positions[idx](1);
            case 2:  return left_foot_positions[idx](2);
            case 3:  return right_foot_positions[idx](0);
            case 4:  return right_foot_positions[idx](1);
            case 5:  return right_foot_positions[idx](2);
            case 6:  return left_foot_velocities[idx](0);
            case 7:  return left_foot_velocities[idx](1);
            case 8:  return left_foot_velocities[idx](2);
            case 9:  return right_foot_velocities[idx](0);
            case 10: return right_foot_velocities[idx](1);
            case 11: return right_foot_velocities[idx](2);
            default: return 0.0;
        }
    };

    // Read and write data rows
    size_t row_idx = 0;
    while (std::getline(input_file, line) && row_idx < num_timesteps) {
        auto fields = split_csv(line);
        // Ensure fields vector is large enough for any new columns
        if (fields.size() < header_fields.size())
            fields.resize(header_fields.size(), "");

        // Write each FK value into its mapped column (existing or new)
        for (size_t k = 0; k < fk_col_names.size(); ++k) {
            std::ostringstream val;
            val << fk_value(k, row_idx);
            fields[fk_col_indices[k]] = val.str();
        }

        // Rebuild row
        std::ostringstream row;
        for (size_t i = 0; i < fields.size(); ++i) {
            if (i > 0) row << ",";
            row << fields[i];
        }
        lines.push_back(row.str());
        row_idx++;
    }
    input_file.close();
    
    // Write back to the same file
    std::ofstream output_file(csv_filename);
    if (!output_file.is_open()) {
        std::cerr << "Error: Could not open CSV file for writing: " << csv_filename << " Skipping." << endl;
        continue;
    }
    
    for (const auto& output_line : lines) {
        output_file << output_line << "\n";
    }
    output_file.close();
    
    cout << "Successfully wrote " << row_idx << " rows with FK data to: " << csv_filename << endl;
    cout << "Added 12 new columns (FK positions and velocities for both feet)" << endl;

    } // end for csv_files loop

    cout << "\nAll files processed." << endl;
    return 0;
}