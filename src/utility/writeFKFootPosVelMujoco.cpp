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
#include <algorithm>
#include <filesystem>
#include "csvParserMujoco.hpp"
#include "RobotModel.hpp"

using std::cout;
using std::endl;

int main() {
    
    // ========================================================================
    // File paths to process — add or remove paths as needed
    // ========================================================================

    const std::filesystem::path data_directory = "../MujocoData";
    std::vector<std::string> csv_files;
    if (!std::filesystem::is_directory(data_directory)) {
        std::cerr << "Error: MuJoCo data directory not found: " << data_directory << endl;
        return -1;
    }
    for (const auto& entry : std::filesystem::directory_iterator(data_directory)) {
        if (entry.is_regular_file() && entry.path().extension() == ".csv") {
            csv_files.push_back(entry.path().string());
        }
    }
    std::sort(csv_files.begin(), csv_files.end());
    if (csv_files.empty()) {
        std::cerr << "Error: No CSV files found in " << data_directory << endl;
        return -1;
    }

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
        
        // Get foot positions (0 = left foot, 1 = right foot)
        Eigen::Vector3d left_pos = robot.get_foot_position(0);
        Eigen::Vector3d right_pos = robot.get_foot_position(1);
        
        // Get foot velocities
        Eigen::Vector3d left_vel = robot.get_foot_velocity(0);
        Eigen::Vector3d right_vel = robot.get_foot_velocity(1);
        
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
