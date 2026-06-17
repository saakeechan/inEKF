/**
 * CSV Parser for G1 Robot Data
 * 
 * This file contains the implementation of ParseCSV function
 * which parses columns from G1LoggingData.csv using header-based extraction
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <cstdlib>
#include <limits>
#include "csvParser.hpp"

using namespace std;

// Helper function to trim whitespace from strings
static std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, (last - first + 1));
}

// Helper function to get column index from header map
static int getColumnIndex(const std::map<std::string, int>& header_map, const std::string& column_name) {
    auto it = header_map.find(column_name);
    if (it != header_map.end()) {
        return it->second;
    }
    return -1; // Column not found
}

// Helper function to safely get value from row by column name
static double getValue(const std::vector<double>& row, const std::map<std::string, int>& header_map, 
                       const std::string& column_name, double default_value = std::numeric_limits<double>::quiet_NaN()) {
    int idx = getColumnIndex(header_map, column_name);
    if (idx >= 0 && idx < static_cast<int>(row.size())) {
        return row[idx];
    }
    return default_value;
}

// Helper macro for conditional getValue (checks if column exists first)
#define SAFE_GET_VALUE(row, header, col_name, default_val) \
    (getColumnIndex(header, col_name) >= 0 ? getValue(row, header, col_name, default_val) : default_val)

// Convenience constant for NaN
static const double NaN = std::numeric_limits<double>::quiet_NaN();

DataStruct ParseCSV(const std::string& csv_filename) {
    
    DataStruct data;
    std::ifstream file(csv_filename);
    
    if (!file.is_open()) {
        std::cerr << "Error: Could not open CSV file: " << csv_filename << std::endl;
        
        // Try with absolute path for debugging
        char abs_path[4096];
        if (realpath(csv_filename.c_str(), abs_path) != NULL) {
            std::cout << "Attempted absolute path: " << abs_path << std::endl;
        } else {
            perror("realpath() error");
        }
        return data;
    }
    
    std::cout << "Successfully opened CSV file: " << csv_filename << std::endl;
    
    // Parse the header line to create column name -> index map
    std::string header_line;
    std::getline(file, header_line);
    
    std::map<std::string, int> header_map;
    std::stringstream header_ss(header_line);
    std::string column_name;
    int col_index = 0;
    
    while (std::getline(header_ss, column_name, ',')) {
        column_name = trim(column_name);
        header_map[column_name] = col_index++;
    }
    
    std::cout << "Parsed " << header_map.size() << " column headers" << std::endl;
    
    // Parse each data row
    std::string line;
    int row_count = 0;
    
    while (std::getline(file, line)) {
        std::vector<double> row_values;
        std::stringstream ss(line);
        std::string value;
        
        // Parse all comma-separated values in this row
        while (std::getline(ss, value, ',')) {
            try {
                row_values.push_back(std::stod(value));
            } catch (const std::exception& e) {
                std::cerr << "Warning: Could not parse value '" << value << "' at row " << row_count << std::endl;
                row_values.push_back(NaN);
            }
        }
        
        // Extract values by column name using the header map (only if columns exist)
        if (getColumnIndex(header_map, "count") >= 0) {
            data.count.push_back(getValue(row_values, header_map, "count"));
        } else {
            data.count.push_back(NaN);
        }
        
        if (getColumnIndex(header_map, "time") >= 0) {
            data.timestamp.push_back(getValue(row_values, header_map, "time"));
        } else {
            data.timestamp.push_back(NaN);
        }
        
        // Base position
        data.pos_x.push_back(getColumnIndex(header_map, "sport_position_0") >= 0 ? getValue(row_values, header_map, "sport_position_0") : NaN);
        data.pos_y.push_back(getColumnIndex(header_map, "sport_position_1") >= 0 ? getValue(row_values, header_map, "sport_position_1") : NaN);
        data.pos_z.push_back(getColumnIndex(header_map, "sport_position_2") >= 0 ? getValue(row_values, header_map, "sport_position_2") : NaN);
        
        // Base orientation quaternion
        data.quat_w.push_back(getColumnIndex(header_map, "base_pose_quat_w") >= 0 ? getValue(row_values, header_map, "base_pose_quat_w") : 1.0);
        data.quat_i.push_back(getColumnIndex(header_map, "base_pose_quat_x") >= 0 ? getValue(row_values, header_map, "base_pose_quat_x") : NaN);
        data.quat_j.push_back(getColumnIndex(header_map, "base_pose_quat_y") >= 0 ? getValue(row_values, header_map, "base_pose_quat_y") : NaN);
        data.quat_k.push_back(getColumnIndex(header_map, "base_pose_quat_z") >= 0 ? getValue(row_values, header_map, "base_pose_quat_z") : NaN);
        
        // Orientation angles
        data.roll.push_back(getColumnIndex(header_map, "roll") >= 0 ? getValue(row_values, header_map, "roll") : NaN);
        data.pitch.push_back(getColumnIndex(header_map, "pitch") >= 0 ? getValue(row_values, header_map, "pitch") : NaN);
        data.yaw.push_back(getColumnIndex(header_map, "yaw") >= 0 ? getValue(row_values, header_map, "yaw") : NaN);
        // Dont have this

        // Base velocity
        data.vel_x.push_back(getColumnIndex(header_map, "sport_velocity_0") >= 0 ? getValue(row_values, header_map, "sport_velocity_0") : NaN);
        data.vel_y.push_back(getColumnIndex(header_map, "sport_velocity_1") >= 0 ? getValue(row_values, header_map, "sport_velocity_1") : NaN);
        data.vel_z.push_back(getColumnIndex(header_map, "sport_velocity_2") >= 0 ? getValue(row_values, header_map, "sport_velocity_2") : NaN);
        
        // Angular velocity
        data.omega_x.push_back(getColumnIndex(header_map, "imu_angvel_world_x") >= 0 ? getValue(row_values, header_map, "imu_angvel_world_x") : NaN);
        data.omega_y.push_back(getColumnIndex(header_map, "imu_angvel_world_y") >= 0 ? getValue(row_values, header_map, "imu_angvel_world_y") : NaN);
        data.omega_z.push_back(getColumnIndex(header_map, "imu_angvel_world_z") >= 0 ? getValue(row_values, header_map, "imu_angvel_world_z") : NaN);
        
        // Linear acceleration
        data.acc_x.push_back(getColumnIndex(header_map, "lowstate_accel_x") >= 0 ? getValue(row_values, header_map, "lowstate_accel_x") : NaN);
        data.acc_y.push_back(getColumnIndex(header_map, "lowstate_accel_y") >= 0 ? getValue(row_values, header_map, "lowstate_accel_y") : NaN);
        data.acc_z.push_back(getColumnIndex(header_map, "lowstate_accel_z") >= 0 ? getValue(row_values, header_map, "lowstate_accel_z") : NaN);
        
        // Angular acceleration
        data.alpha_x.push_back(getColumnIndex(header_map, "alpha_x") >= 0 ? getValue(row_values, header_map, "alpha_x") : NaN);
        data.alpha_y.push_back(getColumnIndex(header_map, "alpha_y") >= 0 ? getValue(row_values, header_map, "alpha_y") : NaN);
        data.alpha_z.push_back(getColumnIndex(header_map, "alpha_z") >= 0 ? getValue(row_values, header_map, "alpha_z") : NaN);
        // Dont have this 

        // Body frame IMU data
        data.acc_body_x.push_back(getColumnIndex(header_map, "lowstate_accel_x") >= 0 ? getValue(row_values, header_map, "lowstate_accel_x") : NaN);
        data.acc_body_y.push_back(getColumnIndex(header_map, "lowstate_accel_y") >= 0 ? getValue(row_values, header_map, "lowstate_accel_y") : NaN);
        data.acc_body_z.push_back(getColumnIndex(header_map, "lowstate_accel_z") >= 0 ? getValue(row_values, header_map, "lowstate_accel_z") : NaN);
        data.gyro_body_x.push_back(getColumnIndex(header_map, "lowstate_gyro_x") >= 0 ? getValue(row_values, header_map, "lowstate_gyro_x") : NaN);
        data.gyro_body_y.push_back(getColumnIndex(header_map, "lowstate_gyro_y") >= 0 ? getValue(row_values, header_map, "lowstate_gyro_y") : NaN);
        data.gyro_body_z.push_back(getColumnIndex(header_map, "lowstate_gyro_z") >= 0 ? getValue(row_values, header_map, "lowstate_gyro_z") : NaN);
        
        // Forward kinematics foot positions
        data.fk_left_foot_pos_x.push_back(SAFE_GET_VALUE(row_values, header_map, "fk_left_foot_pos_x", NaN));
        data.fk_left_foot_pos_y.push_back(SAFE_GET_VALUE(row_values, header_map, "fk_left_foot_pos_y", NaN));
        data.fk_left_foot_pos_z.push_back(SAFE_GET_VALUE(row_values, header_map, "fk_left_foot_pos_z", NaN));
        data.fk_right_foot_pos_x.push_back(SAFE_GET_VALUE(row_values, header_map, "fk_right_foot_pos_x", NaN));
        data.fk_right_foot_pos_y.push_back(SAFE_GET_VALUE(row_values, header_map, "fk_right_foot_pos_y", NaN));
        data.fk_right_foot_pos_z.push_back(SAFE_GET_VALUE(row_values, header_map, "fk_right_foot_pos_z", NaN));
        
        // Forward kinematics foot velocities
        data.fk_left_foot_vel_x.push_back(SAFE_GET_VALUE(row_values, header_map, "fk_left_foot_vel_x", NaN));
        data.fk_left_foot_vel_y.push_back(SAFE_GET_VALUE(row_values, header_map, "fk_left_foot_vel_y", NaN));
        data.fk_left_foot_vel_z.push_back(SAFE_GET_VALUE(row_values, header_map, "fk_left_foot_vel_z", NaN));
        data.fk_right_foot_vel_x.push_back(SAFE_GET_VALUE(row_values, header_map, "fk_right_foot_vel_x", NaN));
        data.fk_right_foot_vel_y.push_back(SAFE_GET_VALUE(row_values, header_map, "fk_right_foot_vel_y", NaN));
        data.fk_right_foot_vel_z.push_back(SAFE_GET_VALUE(row_values, header_map, "fk_right_foot_vel_z", NaN));
        
        // Contact planning and phase
        data.lfoot_contact_plan.push_back(getColumnIndex(header_map, "lfoot-contact-plan") >= 0 ? getValue(row_values, header_map, "lfoot-contact-plan") : NaN);
        data.lfoot_contact.push_back(getColumnIndex(header_map, "sport_foot_force_0") >= 0 ? getValue(row_values, header_map, "sport_foot_force_0") : NaN);
        data.rfoot_contact_plan.push_back(getColumnIndex(header_map, "rfoot-contact-plan") >= 0 ? getValue(row_values, header_map, "rfoot-contact-plan") : NaN);
        data.rfoot_contact.push_back(getColumnIndex(header_map, "sport_foot_force_1") >= 0 ? getValue(row_values, header_map, "sport_foot_force_1") : NaN);
        data.phase_variable.push_back(getColumnIndex(header_map, "phase-variable") >= 0 ? getValue(row_values, header_map, "phase-variable") : NaN);
        
        // Right foot ground reaction forces
        data.rfoot_grf_x.push_back(getColumnIndex(header_map, "rfoot_grf_x") >= 0 ? getValue(row_values, header_map, "rfoot_grf_x") : NaN);
        data.rfoot_grf_y.push_back(getColumnIndex(header_map, "rfoot_grf_y") >= 0 ? getValue(row_values, header_map, "rfoot_grf_y") : NaN);
        data.rfoot_grf_z.push_back(getColumnIndex(header_map, "rfoot_grf_z") >= 0 ? getValue(row_values, header_map, "rfoot_grf_z") : NaN);
        data.rfoot_grt_x.push_back(getColumnIndex(header_map, "rfoot_grt_x") >= 0 ? getValue(row_values, header_map, "rfoot_grt_x") : NaN);
        data.rfoot_grt_y.push_back(getColumnIndex(header_map, "rfoot_grt_y") >= 0 ? getValue(row_values, header_map, "rfoot_grt_y") : NaN);
        data.rfoot_grt_z.push_back(getColumnIndex(header_map, "rfoot_grt_z") >= 0 ? getValue(row_values, header_map, "rfoot_grt_z") : NaN);
        
        // Left foot ground reaction forces
        data.lfoot_grf_x.push_back(getColumnIndex(header_map, "lfoot_grf_x") >= 0 ? getValue(row_values, header_map, "lfoot_grf_x") : NaN);
        data.lfoot_grf_y.push_back(getColumnIndex(header_map, "lfoot_grf_y") >= 0 ? getValue(row_values, header_map, "lfoot_grf_y") : NaN);
        data.lfoot_grf_z.push_back(getColumnIndex(header_map, "lfoot_grf_z") >= 0 ? getValue(row_values, header_map, "lfoot_grf_z") : NaN);
        data.lfoot_grt_x.push_back(getColumnIndex(header_map, "lfoot_grt_x") >= 0 ? getValue(row_values, header_map, "lfoot_grt_x") : NaN);
        data.lfoot_grt_y.push_back(getColumnIndex(header_map, "lfoot_grt_y") >= 0 ? getValue(row_values, header_map, "lfoot_grt_y") : NaN);
        data.lfoot_grt_z.push_back(getColumnIndex(header_map, "lfoot_grt_z") >= 0 ? getValue(row_values, header_map, "lfoot_grt_z") : NaN);
        
        // Right foot pose
        data.rfoot_pos_x.push_back(getColumnIndex(header_map, "rfoot_pos_x") >= 0 ? getValue(row_values, header_map, "rfoot_pos_x") : NaN);
        data.rfoot_pos_y.push_back(getColumnIndex(header_map, "rfoot_pos_y") >= 0 ? getValue(row_values, header_map, "rfoot_pos_y") : NaN);
        data.rfoot_pos_z.push_back(getColumnIndex(header_map, "rfoot_pos_z") >= 0 ? getValue(row_values, header_map, "rfoot_pos_z") : NaN);
        data.rfoot_quat_w.push_back(getColumnIndex(header_map, "rfoot_quat_w") >= 0 ? getValue(row_values, header_map, "rfoot_quat_w") : 1.0);
        data.rfoot_quat_i.push_back(getColumnIndex(header_map, "rfoot_quat_i") >= 0 ? getValue(row_values, header_map, "rfoot_quat_i") : NaN);
        data.rfoot_quat_j.push_back(getColumnIndex(header_map, "rfoot_quat_j") >= 0 ? getValue(row_values, header_map, "rfoot_quat_j") : NaN);
        data.rfoot_quat_k.push_back(getColumnIndex(header_map, "rfoot_quat_k") >= 0 ? getValue(row_values, header_map, "rfoot_quat_k") : NaN);
        
        // Left foot pose
        data.lfoot_pos_x.push_back(getColumnIndex(header_map, "lfoot_pos_x") >= 0 ? getValue(row_values, header_map, "lfoot_pos_x") : NaN);
        data.lfoot_pos_y.push_back(getColumnIndex(header_map, "lfoot_pos_y") >= 0 ? getValue(row_values, header_map, "lfoot_pos_y") : NaN);
        data.lfoot_pos_z.push_back(getColumnIndex(header_map, "lfoot_pos_z") >= 0 ? getValue(row_values, header_map, "lfoot_pos_z") : NaN);
        data.lfoot_quat_w.push_back(getColumnIndex(header_map, "lfoot_quat_w") >= 0 ? getValue(row_values, header_map, "lfoot_quat_w") : 1.0);
        data.lfoot_quat_i.push_back(getColumnIndex(header_map, "lfoot_quat_i") >= 0 ? getValue(row_values, header_map, "lfoot_quat_i") : NaN);
        data.lfoot_quat_j.push_back(getColumnIndex(header_map, "lfoot_quat_j") >= 0 ? getValue(row_values, header_map, "lfoot_quat_j") : NaN);
        data.lfoot_quat_k.push_back(getColumnIndex(header_map, "lfoot_quat_k") >= 0 ? getValue(row_values, header_map, "lfoot_quat_k") : NaN);
        
        // Joint positions (with existence checks)
        data.joint_pos_left_hip_pitch.push_back(getColumnIndex(header_map, "joint_left_hip_pitch_joint_pos") >= 0 ? getValue(row_values, header_map, "joint_left_hip_pitch_joint_pos") : NaN);
        data.joint_pos_right_hip_pitch.push_back(getColumnIndex(header_map, "joint_right_hip_pitch_joint_pos") >= 0 ? getValue(row_values, header_map, "joint_right_hip_pitch_joint_pos") : NaN);
        data.joint_pos_waist_yaw.push_back(getColumnIndex(header_map, "joint_waist_yaw_joint_pos") >= 0 ? getValue(row_values, header_map, "joint_waist_yaw_joint_pos") : NaN);
        data.joint_pos_left_hip_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hip_roll_joint_pos", NaN));
        data.joint_pos_right_hip_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hip_roll_joint_pos", NaN));
        data.joint_pos_waist_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_waist_roll_joint_pos", NaN));
        data.joint_pos_left_hip_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hip_yaw_joint_pos", NaN));
        data.joint_pos_right_hip_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hip_yaw_joint_pos", NaN));
        data.joint_pos_waist_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_waist_pitch_joint_pos", NaN));
        data.joint_pos_left_knee.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_knee_joint_pos", NaN));
        data.joint_pos_right_knee.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_knee_joint_pos", NaN));
        data.joint_pos_left_shoulder_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_shoulder_pitch_joint_pos", NaN));
        data.joint_pos_right_shoulder_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_shoulder_pitch_joint_pos", NaN));
        data.joint_pos_left_ankle_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_ankle_pitch_joint_pos", NaN));
        data.joint_pos_right_ankle_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_ankle_pitch_joint_pos", NaN));
        data.joint_pos_left_shoulder_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_shoulder_roll_joint_pos", NaN));
        data.joint_pos_right_shoulder_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_shoulder_roll_joint_pos", NaN));
        data.joint_pos_left_ankle_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_ankle_roll_joint_pos", NaN));
        data.joint_pos_right_ankle_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_ankle_roll_joint_pos", NaN));
        data.joint_pos_left_shoulder_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_shoulder_yaw_joint_pos", NaN));
        data.joint_pos_right_shoulder_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_shoulder_yaw_joint_pos", NaN));
        data.joint_pos_left_elbow.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_elbow_joint_pos", NaN));
        data.joint_pos_right_elbow.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_elbow_joint_pos", NaN));
        data.joint_pos_left_wrist_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_wrist_roll_joint_pos", NaN));
        data.joint_pos_right_wrist_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_wrist_roll_joint_pos", NaN));
        data.joint_pos_left_wrist_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_wrist_pitch_joint_pos", NaN));
        data.joint_pos_right_wrist_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_wrist_pitch_joint_pos", NaN));
        data.joint_pos_left_wrist_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_wrist_yaw_joint_pos", NaN));
        data.joint_pos_right_wrist_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_wrist_yaw_joint_pos", NaN));
        data.joint_pos_left_hand_index_0.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hand_index_0_joint_pos", NaN));
        data.joint_pos_left_hand_middle_0.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hand_middle_0_joint_pos", NaN));
        data.joint_pos_left_hand_thumb_0.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hand_thumb_0_joint_pos", NaN));
        data.joint_pos_right_hand_index_0.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hand_index_0_joint_pos", NaN));
        data.joint_pos_right_hand_middle_0.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hand_middle_0_joint_pos", NaN));
        data.joint_pos_right_hand_thumb_0.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hand_thumb_0_joint_pos", NaN));
        data.joint_pos_left_hand_index_1.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hand_index_1_joint_pos", NaN));
        data.joint_pos_left_hand_middle_1.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hand_middle_1_joint_pos", NaN));
        data.joint_pos_left_hand_thumb_1.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hand_thumb_1_joint_pos", NaN));
        data.joint_pos_right_hand_index_1.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hand_index_1_joint_pos", NaN));
        data.joint_pos_right_hand_middle_1.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hand_middle_1_joint_pos", NaN));
        data.joint_pos_right_hand_thumb_1.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hand_thumb_1_joint_pos", NaN));
        data.joint_pos_left_hand_thumb_2.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hand_thumb_2_joint_pos", NaN));
        data.joint_pos_right_hand_thumb_2.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hand_thumb_2_joint_pos", NaN));
        
        // Joint velocities (with column existence checks via SAFE_GET_VALUE macro)
        data.joint_vel_left_hip_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hip_pitch_joint_vel", NaN));
        data.joint_vel_right_hip_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hip_pitch_joint_vel", NaN));
        data.joint_vel_waist_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_waist_yaw_joint_vel", NaN));
        data.joint_vel_left_hip_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hip_roll_joint_vel", NaN));
        data.joint_vel_right_hip_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hip_roll_joint_vel", NaN));
        data.joint_vel_waist_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_waist_roll_joint_vel", NaN));
        data.joint_vel_left_hip_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hip_yaw_joint_vel", NaN));
        data.joint_vel_right_hip_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hip_yaw_joint_vel", NaN));
        data.joint_vel_waist_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_waist_pitch_joint_vel", NaN));
        data.joint_vel_left_knee.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_knee_joint_vel", NaN));
        data.joint_vel_right_knee.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_knee_joint_vel", NaN));
        data.joint_vel_left_shoulder_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_shoulder_pitch_joint_vel", NaN));
        data.joint_vel_right_shoulder_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_shoulder_pitch_joint_vel", NaN));
        data.joint_vel_left_ankle_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_ankle_pitch_joint_vel", NaN));
        data.joint_vel_right_ankle_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_ankle_pitch_joint_vel", NaN));
        data.joint_vel_left_shoulder_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_shoulder_roll_joint_vel", NaN));
        data.joint_vel_right_shoulder_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_shoulder_roll_joint_vel", NaN));
        data.joint_vel_left_ankle_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_ankle_roll_joint_vel", NaN));
        data.joint_vel_right_ankle_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_ankle_roll_joint_vel", NaN));
        data.joint_vel_left_shoulder_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_shoulder_yaw_joint_vel", NaN));
        data.joint_vel_right_shoulder_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_shoulder_yaw_joint_vel", NaN));
        data.joint_vel_left_elbow.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_elbow_joint_vel", NaN));
        data.joint_vel_right_elbow.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_elbow_joint_vel", NaN));
        data.joint_vel_left_wrist_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_wrist_roll_joint_vel", NaN));
        data.joint_vel_right_wrist_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_wrist_roll_joint_vel", NaN));
        data.joint_vel_left_wrist_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_wrist_pitch_joint_vel", NaN));
        data.joint_vel_right_wrist_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_wrist_pitch_joint_vel", NaN));
        data.joint_vel_left_wrist_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_wrist_yaw_joint_vel", NaN));
        data.joint_vel_right_wrist_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_wrist_yaw_joint_vel", NaN));
        data.joint_vel_left_hand_index_0.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hand_index_0_joint_vel", NaN));
        data.joint_vel_left_hand_middle_0.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hand_middle_0_joint_vel", NaN));
        data.joint_vel_left_hand_thumb_0.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hand_thumb_0_joint_vel", NaN));
        data.joint_vel_right_hand_index_0.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hand_index_0_joint_vel", NaN));
        data.joint_vel_right_hand_middle_0.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hand_middle_0_joint_vel", NaN));
        data.joint_vel_right_hand_thumb_0.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hand_thumb_0_joint_vel", NaN));
        data.joint_vel_left_hand_index_1.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hand_index_1_joint_vel", NaN));
        data.joint_vel_left_hand_middle_1.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hand_middle_1_joint_vel", NaN));
        data.joint_vel_left_hand_thumb_1.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hand_thumb_1_joint_vel", NaN));
        data.joint_vel_right_hand_index_1.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hand_index_1_joint_vel", NaN));
        data.joint_vel_right_hand_middle_1.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hand_middle_1_joint_vel", NaN));
        data.joint_vel_right_hand_thumb_1.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hand_thumb_1_joint_vel", NaN));
        data.joint_vel_left_hand_thumb_2.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hand_thumb_2_joint_vel", NaN));
        data.joint_vel_right_hand_thumb_2.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hand_thumb_2_joint_vel", NaN));
        
        // Joint torques
        data.joint_torque_left_hip_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hip_pitch_joint_eff", NaN));
        data.joint_torque_right_hip_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hip_pitch_joint_eff", NaN));
        data.joint_torque_waist_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_waist_yaw_joint_eff", NaN));
        data.joint_torque_left_hip_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hip_roll_joint_eff", NaN));
        data.joint_torque_right_hip_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hip_roll_joint_eff", NaN));
        data.joint_torque_waist_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_waist_roll_joint_eff", NaN));
        data.joint_torque_left_hip_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hip_yaw_joint_eff", NaN));
        data.joint_torque_right_hip_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hip_yaw_joint_eff", NaN));
        data.joint_torque_waist_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_waist_pitch_joint_eff", NaN));
        data.joint_torque_left_knee.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_knee_joint_eff", NaN));
        data.joint_torque_right_knee.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_knee_joint_eff", NaN));
        data.joint_torque_left_shoulder_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_shoulder_pitch_joint_eff", NaN));
        data.joint_torque_right_shoulder_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_shoulder_pitch_joint_eff", NaN));
        data.joint_torque_left_ankle_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_ankle_pitch_joint_eff", NaN));
        data.joint_torque_right_ankle_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_ankle_pitch_joint_eff", NaN));
        data.joint_torque_left_shoulder_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_shoulder_roll_joint_eff", NaN));
        data.joint_torque_right_shoulder_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_shoulder_roll_joint_eff", NaN));
        data.joint_torque_left_ankle_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_ankle_roll_joint_eff", NaN));
        data.joint_torque_right_ankle_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_ankle_roll_joint_eff", NaN));
        data.joint_torque_left_shoulder_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_shoulder_yaw_joint_eff", NaN));
        data.joint_torque_right_shoulder_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_shoulder_yaw_joint_eff", NaN));
        data.joint_torque_left_elbow.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_elbow_joint_eff", NaN));
        data.joint_torque_right_elbow.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_elbow_joint_eff", NaN));
        data.joint_torque_left_wrist_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_wrist_roll_joint_eff", NaN));
        data.joint_torque_right_wrist_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_wrist_roll_joint_eff", NaN));
        data.joint_torque_left_wrist_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_wrist_pitch_joint_eff", NaN));
        data.joint_torque_right_wrist_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_wrist_pitch_joint_eff", NaN));
        data.joint_torque_left_wrist_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_wrist_yaw_joint_eff", NaN));
        data.joint_torque_right_wrist_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_wrist_yaw_joint_eff", NaN));
        data.joint_torque_left_hand_index_0.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hand_index_0_joint_eff", NaN));
        data.joint_torque_left_hand_middle_0.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hand_middle_0_joint_eff", NaN));
        data.joint_torque_left_hand_thumb_0.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hand_thumb_0_joint_eff", NaN));
        data.joint_torque_right_hand_index_0.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hand_index_0_joint_eff", NaN));
        data.joint_torque_right_hand_middle_0.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hand_middle_0_joint_eff", NaN));
        data.joint_torque_right_hand_thumb_0.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hand_thumb_0_joint_eff", NaN));
        data.joint_torque_left_hand_index_1.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hand_index_1_joint_eff", NaN));
        data.joint_torque_left_hand_middle_1.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hand_middle_1_joint_eff", NaN));
        data.joint_torque_left_hand_thumb_1.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hand_thumb_1_joint_eff", NaN));
        data.joint_torque_right_hand_index_1.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hand_index_1_joint_eff", NaN));
        data.joint_torque_right_hand_middle_1.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hand_middle_1_joint_eff", NaN));
        data.joint_torque_right_hand_thumb_1.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hand_thumb_1_joint_eff", NaN));
        data.joint_torque_left_hand_thumb_2.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_left_hand_thumb_2_joint_eff", NaN));
        data.joint_torque_right_hand_thumb_2.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_right_hand_thumb_2_joint_eff", NaN));
        
        // Joint commands
        data.joint_cmd_left_hip_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_left_hip_pitch_joint_pos", NaN));
        data.joint_cmd_right_hip_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_right_hip_pitch_joint_pos", NaN));
        data.joint_cmd_waist_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_waist_yaw_joint_pos", NaN));
        data.joint_cmd_left_hip_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_left_hip_roll_joint_pos", NaN));
        data.joint_cmd_right_hip_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_right_hip_roll_joint_pos", NaN));
        data.joint_cmd_waist_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_waist_roll_joint_pos", NaN));
        data.joint_cmd_left_hip_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_left_hip_yaw_joint_pos", NaN));
        data.joint_cmd_right_hip_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_right_hip_yaw_joint_pos", NaN));
        data.joint_cmd_waist_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_waist_pitch_joint_pos", NaN));
        data.joint_cmd_left_knee.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_left_knee_joint_pos", NaN));
        data.joint_cmd_right_knee.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_right_knee_joint_pos", NaN));
        data.joint_cmd_left_shoulder_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_left_shoulder_pitch_joint_pos", NaN));
        data.joint_cmd_right_shoulder_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_right_shoulder_pitch_joint_pos", NaN));
        data.joint_cmd_left_ankle_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_left_ankle_pitch_joint_pos", NaN));
        data.joint_cmd_right_ankle_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_right_ankle_pitch_joint_pos", NaN));
        data.joint_cmd_left_shoulder_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_left_shoulder_roll_joint_pos", NaN));
        data.joint_cmd_right_shoulder_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_right_shoulder_roll_joint_pos", NaN));
        data.joint_cmd_left_ankle_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_left_ankle_roll_joint_pos", NaN));
        data.joint_cmd_right_ankle_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_right_ankle_roll_joint_pos", NaN));
        data.joint_cmd_left_shoulder_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_left_shoulder_yaw_joint_pos", NaN));
        data.joint_cmd_right_shoulder_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_right_shoulder_yaw_joint_pos", NaN));
        data.joint_cmd_left_elbow.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_left_elbow_joint_pos", NaN));
        data.joint_cmd_right_elbow.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_right_elbow_joint_pos", NaN));
        data.joint_cmd_left_wrist_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_left_wrist_roll_joint_pos", NaN));
        data.joint_cmd_right_wrist_roll.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_right_wrist_roll_joint_pos", NaN));
        data.joint_cmd_left_wrist_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_left_wrist_pitch_joint_pos", NaN));
        data.joint_cmd_right_wrist_pitch.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_right_wrist_pitch_joint_pos", NaN));
        data.joint_cmd_left_wrist_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_left_wrist_yaw_joint_pos", NaN));
        data.joint_cmd_right_wrist_yaw.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_right_wrist_yaw_joint_pos", NaN));
        data.joint_cmd_left_hand_index_0.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_left_hand_index_0_joint_pos", NaN));
        data.joint_cmd_left_hand_middle_0.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_left_hand_middle_0_joint_pos", NaN));
        data.joint_cmd_left_hand_thumb_0.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_left_hand_thumb_0_joint_pos", NaN));
        data.joint_cmd_right_hand_index_0.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_right_hand_index_0_joint_pos", NaN));
        data.joint_cmd_right_hand_middle_0.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_right_hand_middle_0_joint_pos", NaN));
        data.joint_cmd_right_hand_thumb_0.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_right_hand_thumb_0_joint_pos", NaN));
        data.joint_cmd_left_hand_index_1.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_left_hand_index_1_joint_pos", NaN));
        data.joint_cmd_left_hand_middle_1.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_left_hand_middle_1_joint_pos", NaN));
        data.joint_cmd_left_hand_thumb_1.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_left_hand_thumb_1_joint_pos", NaN));
        data.joint_cmd_right_hand_index_1.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_right_hand_index_1_joint_pos", NaN));
        data.joint_cmd_right_hand_middle_1.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_right_hand_middle_1_joint_pos", NaN));
        data.joint_cmd_right_hand_thumb_1.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_right_hand_thumb_1_joint_pos", NaN));
        data.joint_cmd_left_hand_thumb_2.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_left_hand_thumb_2_joint_pos", NaN));
        data.joint_cmd_right_hand_thumb_2.push_back(SAFE_GET_VALUE(row_values, header_map, "joint_cmd_right_hand_thumb_2_joint_pos", NaN));
        
        // Command velocities and stability
        data.cmd_vel_x.push_back(SAFE_GET_VALUE(row_values, header_map, "cmd_vel_x", NaN));
        data.cmd_vel_y.push_back(SAFE_GET_VALUE(row_values, header_map, "cmd_vel_y", NaN));
        data.cmd_vel_z.push_back(SAFE_GET_VALUE(row_values, header_map, "cmd_vel_z", NaN));
        data.cmd_w.push_back(SAFE_GET_VALUE(row_values, header_map, "cmd_turning", NaN));
        data.stability.push_back(SAFE_GET_VALUE(row_values, header_map, "stability", NaN));
        
        row_count++;
    }
    
    file.close();
    data.num_rows = row_count;
    
    std::cout << "Parsed " << row_count << " rows from G1 CSV file" << std::endl;
    std::cout << "Total columns parsed: " << header_map.size() << " (from header)" << std::endl;
    
    // Now populate the 2D arrays for easier access
    std::cout << "Populating 2D encoder and contact arrays..." << std::endl;
    
    data.encoders.resize(row_count, std::vector<double>(data.num_encoders));
    data.encoderdots.resize(row_count, std::vector<double>(data.num_encoders));
    data.motor_torques.resize(row_count, std::vector<double>(data.num_encoders));
    data.motor_pos_cmd.resize(row_count, std::vector<double>(data.num_encoders));
    data.CMD_V.resize(row_count, std::vector<double>(3));
    data.contact_state.resize(row_count, std::vector<bool>(data.num_contacts));
    
    std::cout << "  Resized encoders to " << data.encoders.size() << " rows x " << data.num_encoders << " cols" << std::endl;
    
    // Map according to G1 joint order (q[0] to q[28])
    for (size_t i = 0; i < row_count; i++) {
        // Left leg (q[0-5])
        data.encoders[i][0] = data.joint_pos_left_hip_pitch[i];
        data.encoders[i][1] = data.joint_pos_left_hip_roll[i];
        data.encoders[i][2] = data.joint_pos_left_hip_yaw[i];
        data.encoders[i][3] = data.joint_pos_left_knee[i];
        data.encoders[i][4] = data.joint_pos_left_ankle_pitch[i];
        data.encoders[i][5] = data.joint_pos_left_ankle_roll[i];
        
        // Right leg (q[6-11])
        data.encoders[i][6] = data.joint_pos_right_hip_pitch[i];
        data.encoders[i][7] = data.joint_pos_right_hip_roll[i];
        data.encoders[i][8] = data.joint_pos_right_hip_yaw[i];
        data.encoders[i][9] = data.joint_pos_right_knee[i];
        data.encoders[i][10] = data.joint_pos_right_ankle_pitch[i];
        data.encoders[i][11] = data.joint_pos_right_ankle_roll[i];
        
        // Waist (q[12-14])
        data.encoders[i][12] = data.joint_pos_waist_yaw[i];
        data.encoders[i][13] = data.joint_pos_waist_roll[i];
        data.encoders[i][14] = data.joint_pos_waist_pitch[i];
        
        // Left arm (q[15-21])
        data.encoders[i][15] = data.joint_pos_left_shoulder_pitch[i];
        data.encoders[i][16] = data.joint_pos_left_shoulder_roll[i];
        data.encoders[i][17] = data.joint_pos_left_shoulder_yaw[i];
        data.encoders[i][18] = data.joint_pos_left_elbow[i];
        data.encoders[i][19] = data.joint_pos_left_wrist_roll[i];
        data.encoders[i][20] = data.joint_pos_left_wrist_pitch[i];
        data.encoders[i][21] = data.joint_pos_left_wrist_yaw[i];
        
        // Right arm (q[22-28])
        data.encoders[i][22] = data.joint_pos_right_shoulder_pitch[i];
        data.encoders[i][23] = data.joint_pos_right_shoulder_roll[i];
        data.encoders[i][24] = data.joint_pos_right_shoulder_yaw[i];
        data.encoders[i][25] = data.joint_pos_right_elbow[i];
        data.encoders[i][26] = data.joint_pos_right_wrist_roll[i];
        data.encoders[i][27] = data.joint_pos_right_wrist_pitch[i];
        data.encoders[i][28] = data.joint_pos_right_wrist_yaw[i];
        
        // Left leg velocities (v[0-5])
        data.encoderdots[i][0] = data.joint_vel_left_hip_pitch[i];
        data.encoderdots[i][1] = data.joint_vel_left_hip_roll[i];
        data.encoderdots[i][2] = data.joint_vel_left_hip_yaw[i];
        data.encoderdots[i][3] = data.joint_vel_left_knee[i];
        data.encoderdots[i][4] = data.joint_vel_left_ankle_pitch[i];
        data.encoderdots[i][5] = data.joint_vel_left_ankle_roll[i];
        
        // Right leg velocities (v[6-11])
        data.encoderdots[i][6] = data.joint_vel_right_hip_pitch[i];
        data.encoderdots[i][7] = data.joint_vel_right_hip_roll[i];
        data.encoderdots[i][8] = data.joint_vel_right_hip_yaw[i];
        data.encoderdots[i][9] = data.joint_vel_right_knee[i];
        data.encoderdots[i][10] = data.joint_vel_right_ankle_pitch[i];
        data.encoderdots[i][11] = data.joint_vel_right_ankle_roll[i];
        
        // Waist velocities (v[12-14])
        data.encoderdots[i][12] = data.joint_vel_waist_yaw[i];
        data.encoderdots[i][13] = data.joint_vel_waist_roll[i];
        data.encoderdots[i][14] = data.joint_vel_waist_pitch[i];
        
        // Left arm velocities (v[15-21])
        data.encoderdots[i][15] = data.joint_vel_left_shoulder_pitch[i];
        data.encoderdots[i][16] = data.joint_vel_left_shoulder_roll[i];
        data.encoderdots[i][17] = data.joint_vel_left_shoulder_yaw[i];
        data.encoderdots[i][18] = data.joint_vel_left_elbow[i];
        data.encoderdots[i][19] = data.joint_vel_left_wrist_roll[i];
        data.encoderdots[i][20] = data.joint_vel_left_wrist_pitch[i];
        data.encoderdots[i][21] = data.joint_vel_left_wrist_yaw[i];
        
        // Right arm velocities (v[22-28])
        data.encoderdots[i][22] = data.joint_vel_right_shoulder_pitch[i];
        data.encoderdots[i][23] = data.joint_vel_right_shoulder_roll[i];
        data.encoderdots[i][24] = data.joint_vel_right_shoulder_yaw[i];
        data.encoderdots[i][25] = data.joint_vel_right_elbow[i];
        data.encoderdots[i][26] = data.joint_vel_right_wrist_roll[i];
        data.encoderdots[i][27] = data.joint_vel_right_wrist_pitch[i];
        data.encoderdots[i][28] = data.joint_vel_right_wrist_yaw[i];
        
        // Motor torques
        data.motor_torques[i][0] = data.joint_torque_left_hip_pitch[i];
        data.motor_torques[i][1] = data.joint_torque_left_hip_roll[i];
        data.motor_torques[i][2] = data.joint_torque_left_hip_yaw[i];
        data.motor_torques[i][3] = data.joint_torque_left_knee[i];
        data.motor_torques[i][4] = data.joint_torque_left_ankle_pitch[i];
        data.motor_torques[i][5] = data.joint_torque_left_ankle_roll[i];
        data.motor_torques[i][6] = data.joint_torque_right_hip_pitch[i];
        data.motor_torques[i][7] = data.joint_torque_right_hip_roll[i];
        data.motor_torques[i][8] = data.joint_torque_right_hip_yaw[i];
        data.motor_torques[i][9] = data.joint_torque_right_knee[i];
        data.motor_torques[i][10] = data.joint_torque_right_ankle_pitch[i];
        data.motor_torques[i][11] = data.joint_torque_right_ankle_roll[i];
        data.motor_torques[i][12] = data.joint_torque_waist_yaw[i];
        data.motor_torques[i][13] = data.joint_torque_waist_roll[i];
        data.motor_torques[i][14] = data.joint_torque_waist_pitch[i];
        data.motor_torques[i][15] = data.joint_torque_left_shoulder_pitch[i];
        data.motor_torques[i][16] = data.joint_torque_left_shoulder_roll[i];
        data.motor_torques[i][17] = data.joint_torque_left_shoulder_yaw[i];
        data.motor_torques[i][18] = data.joint_torque_left_elbow[i];
        data.motor_torques[i][19] = data.joint_torque_left_wrist_roll[i];
        data.motor_torques[i][20] = data.joint_torque_left_wrist_pitch[i];
        data.motor_torques[i][21] = data.joint_torque_left_wrist_yaw[i];
        data.motor_torques[i][22] = data.joint_torque_right_shoulder_pitch[i];
        data.motor_torques[i][23] = data.joint_torque_right_shoulder_roll[i];
        data.motor_torques[i][24] = data.joint_torque_right_shoulder_yaw[i];
        data.motor_torques[i][25] = data.joint_torque_right_elbow[i];
        data.motor_torques[i][26] = data.joint_torque_right_wrist_roll[i];
        data.motor_torques[i][27] = data.joint_torque_right_wrist_pitch[i];
        data.motor_torques[i][28] = data.joint_torque_right_wrist_yaw[i];
        
        // Motor position commands
        data.motor_pos_cmd[i][0] = data.joint_cmd_left_hip_pitch[i];
        data.motor_pos_cmd[i][1] = data.joint_cmd_left_hip_roll[i];
        data.motor_pos_cmd[i][2] = data.joint_cmd_left_hip_yaw[i];
        data.motor_pos_cmd[i][3] = data.joint_cmd_left_knee[i];
        data.motor_pos_cmd[i][4] = data.joint_cmd_left_ankle_pitch[i];
        data.motor_pos_cmd[i][5] = data.joint_cmd_left_ankle_roll[i];
        data.motor_pos_cmd[i][6] = data.joint_cmd_right_hip_pitch[i];
        data.motor_pos_cmd[i][7] = data.joint_cmd_right_hip_roll[i];
        data.motor_pos_cmd[i][8] = data.joint_cmd_right_hip_yaw[i];
        data.motor_pos_cmd[i][9] = data.joint_cmd_right_knee[i];
        data.motor_pos_cmd[i][10] = data.joint_cmd_right_ankle_pitch[i];
        data.motor_pos_cmd[i][11] = data.joint_cmd_right_ankle_roll[i];
        data.motor_pos_cmd[i][12] = data.joint_cmd_waist_yaw[i];
        data.motor_pos_cmd[i][13] = data.joint_cmd_waist_roll[i];
        data.motor_pos_cmd[i][14] = data.joint_cmd_waist_pitch[i];
        data.motor_pos_cmd[i][15] = data.joint_cmd_left_shoulder_pitch[i];
        data.motor_pos_cmd[i][16] = data.joint_cmd_left_shoulder_roll[i];
        data.motor_pos_cmd[i][17] = data.joint_cmd_left_shoulder_yaw[i];
        data.motor_pos_cmd[i][18] = data.joint_cmd_left_elbow[i];
        data.motor_pos_cmd[i][19] = data.joint_cmd_left_wrist_roll[i];
        data.motor_pos_cmd[i][20] = data.joint_cmd_left_wrist_pitch[i];
        data.motor_pos_cmd[i][21] = data.joint_cmd_left_wrist_yaw[i];
        data.motor_pos_cmd[i][22] = data.joint_cmd_right_shoulder_pitch[i];
        data.motor_pos_cmd[i][23] = data.joint_cmd_right_shoulder_roll[i];
        data.motor_pos_cmd[i][24] = data.joint_cmd_right_shoulder_yaw[i];
        data.motor_pos_cmd[i][25] = data.joint_cmd_right_elbow[i];
        data.motor_pos_cmd[i][26] = data.joint_cmd_right_wrist_roll[i];
        data.motor_pos_cmd[i][27] = data.joint_cmd_right_wrist_pitch[i];
        data.motor_pos_cmd[i][28] = data.joint_cmd_right_wrist_yaw[i];
        
        // Command velocity
        data.CMD_V[i][0] = data.cmd_vel_x[i];
        data.CMD_V[i][1] = data.cmd_vel_y[i];
        data.CMD_V[i][2] = data.cmd_vel_z[i];
        
        // Contact states
        data.contact_state[i][0] = data.lfoot_contact[i];
        data.contact_state[i][1] = data.rfoot_contact[i];
    }
    
    std::cout << "Successfully populated 2D arrays" << std::endl;
    
    return data;
}
