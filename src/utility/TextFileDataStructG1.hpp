/**
 * TextFileDataStruct.hpp
 * 
 * Universal data structure for robot sensor data across different platforms.
 * Provides a common interface for IMU data, encoders, and contact states
 * that can be populated from any robot-specific CSV parser (Digit, G1, Go2, etc.)
 * 
 * This allows the estimator to use a single data structure regardless of
 * which robot is being used, eliminating the need to change header names
 * when switching between robots.
 */

#ifndef TextFileDataStruct_HPP
#define TextFileDataStruct_HPP

#include <eigen3/Eigen/Dense>
#include <vector>
#include <cstddef>
#include <string>
#include "G1/csvParserG1.hpp"

/**
 * Universal data structure for robot sensor data
 * Contains IMU measurements, joint encoders, encoder velocities, and contact states
 * Can be populated from any robot-specific parser (Digit, G1, Go2, etc.)
 */
class TextFileDataStruct {
public:
    // CSV Parser data source
    G1DataStruct robot_data; // <- This can be replaced with the appropriate data structure for other robots (e.g., DigitDataStruct, Go2DataStruct)
    
    // Configuration
    size_t num_encoders = 29; // Number of active encoders
    size_t num_contacts = 2;
    
    // Timestamp data
    std::vector<double> timestamp;
    
    // IMU data
    std::vector<double> gyro_x, gyro_y, gyro_z;
    std::vector<double> acc_x, acc_y, acc_z;
    
    // Joint encoder positions - 2D array [num_rows x num_encoders]
    std::vector<std::vector<double>> encoders;
    
    // Joint encoder velocities - 2D array [num_rows x num_encoders]
    std::vector<std::vector<double>> encoderdots;

    std::vector<std::vector<double>> motor_torques; // 2D array [num_rows x num_motors]
    
    // Base position data
    std::vector<double> pos_x, pos_y, pos_z;
    
    // Base velocity data
    std::vector<double> vel_x, vel_y, vel_z;

    std::vector<double> quat_w, quat_i, quat_j, quat_k;
    
    // Contact states - 2D array [num_rows x num_contacts]
    std::vector<std::vector<bool>> contact_state;
    
    // Number of data points (rows/timestamps)
    size_t num_rows = 0;
    
    // Constructor
    TextFileDataStruct() {}
    
    // Simple direct assignment from csvParser
    void loadFromParser(const std::string& filename) {
        robot_data = ParseG1CSV(filename); // This can be switched to a different parser based on the robot type
        
        // Direct assignment
        timestamp = robot_data.timestamp;
        gyro_x = robot_data.gyro_body_x;
        gyro_y = robot_data.gyro_body_y;
        gyro_z = robot_data.gyro_body_z;
        acc_x = robot_data.acc_body_x;
        acc_y = robot_data.acc_body_y;
        acc_z = robot_data.acc_body_z;
        pos_x = robot_data.pos_x;
        pos_y = robot_data.pos_y;
        pos_z = robot_data.pos_z;
        vel_x = robot_data.vel_x;
        vel_y = robot_data.vel_y;
        vel_z = robot_data.vel_z;
        quat_w = robot_data.quat_w;
        quat_i = robot_data.quat_i;
        quat_j = robot_data.quat_j;
        quat_k = robot_data.quat_k;
        
        num_rows = robot_data.num_rows;

        
        // Encoders - 2D array [num_rows x num_encoders]
        encoders.resize(num_rows, std::vector<double>(num_encoders));
        encoderdots.resize(num_rows, std::vector<double>(num_encoders));
        motor_torques.resize(num_rows, std::vector<double>(num_encoders)); // Assuming 1 torque per motor, adjust if different

        
        std::cout << "  Resized encoders to " << encoders.size() << " rows x " << num_encoders << " cols" << std::endl;
        
        // Map according to G1 joint order (q[0] to q[28])
        for (size_t i = 0; i < num_rows; i++) {
            // Left leg (q[0-5])
            encoders[i][0] = robot_data.joint_pos_left_hip_pitch[i];
            encoders[i][1] = robot_data.joint_pos_left_hip_roll[i];
            encoders[i][2] = robot_data.joint_pos_left_hip_yaw[i];
            encoders[i][3] = robot_data.joint_pos_left_knee[i];
            encoders[i][4] = robot_data.joint_pos_left_ankle_pitch[i];
            encoders[i][5] = robot_data.joint_pos_left_ankle_roll[i];
            
            // Right leg (q[6-11])
            encoders[i][6] = robot_data.joint_pos_right_hip_pitch[i];
            encoders[i][7] = robot_data.joint_pos_right_hip_roll[i];
            encoders[i][8] = robot_data.joint_pos_right_hip_yaw[i];
            encoders[i][9] = robot_data.joint_pos_right_knee[i];
            encoders[i][10] = robot_data.joint_pos_right_ankle_pitch[i];
            encoders[i][11] = robot_data.joint_pos_right_ankle_roll[i];
            
            // Waist (q[12-14])
            encoders[i][12] = robot_data.joint_pos_waist_yaw[i];
            encoders[i][13] = robot_data.joint_pos_waist_roll[i];
            encoders[i][14] = robot_data.joint_pos_waist_pitch[i];
            
            // Left arm (q[15-21])
            encoders[i][15] = robot_data.joint_pos_left_shoulder_pitch[i];
            encoders[i][16] = robot_data.joint_pos_left_shoulder_roll[i];
            encoders[i][17] = robot_data.joint_pos_left_shoulder_yaw[i];
            encoders[i][18] = robot_data.joint_pos_left_elbow[i];
            encoders[i][19] = robot_data.joint_pos_left_wrist_roll[i];
            encoders[i][20] = robot_data.joint_pos_left_wrist_pitch[i];
            encoders[i][21] = robot_data.joint_pos_left_wrist_yaw[i];
            
            // Right arm (q[22-28])
            encoders[i][22] = robot_data.joint_pos_right_shoulder_pitch[i];
            encoders[i][23] = robot_data.joint_pos_right_shoulder_roll[i];
            encoders[i][24] = robot_data.joint_pos_right_shoulder_yaw[i];
            encoders[i][25] = robot_data.joint_pos_right_elbow[i];
            encoders[i][26] = robot_data.joint_pos_right_wrist_roll[i];
            encoders[i][27] = robot_data.joint_pos_right_wrist_pitch[i];
            encoders[i][28] = robot_data.joint_pos_right_wrist_yaw[i];
            
            // -------------------------------------------------------//

            // Left leg velocities (v[0-5])
            encoderdots[i][0] = robot_data.joint_vel_left_hip_pitch[i];
            encoderdots[i][1] = robot_data.joint_vel_left_hip_roll[i];
            encoderdots[i][2] = robot_data.joint_vel_left_hip_yaw[i];
            encoderdots[i][3] = robot_data.joint_vel_left_knee[i];
            encoderdots[i][4] = robot_data.joint_vel_left_ankle_pitch[i];
            encoderdots[i][5] = robot_data.joint_vel_left_ankle_roll[i];
            
            // Right leg velocities (v[6-11])
            encoderdots[i][6] = robot_data.joint_vel_right_hip_pitch[i];
            encoderdots[i][7] = robot_data.joint_vel_right_hip_roll[i];
            encoderdots[i][8] = robot_data.joint_vel_right_hip_yaw[i];
            encoderdots[i][9] = robot_data.joint_vel_right_knee[i];
            encoderdots[i][10] = robot_data.joint_vel_right_ankle_pitch[i];
            encoderdots[i][11] = robot_data.joint_vel_right_ankle_roll[i];
            
            // Waist velocities (v[12-14])
            encoderdots[i][12] = robot_data.joint_vel_waist_yaw[i];
            encoderdots[i][13] = robot_data.joint_vel_waist_roll[i];
            encoderdots[i][14] = robot_data.joint_vel_waist_pitch[i];
            
            // Left arm velocities (v[15-21])
            encoderdots[i][15] = robot_data.joint_vel_left_shoulder_pitch[i];
            encoderdots[i][16] = robot_data.joint_vel_left_shoulder_roll[i];
            encoderdots[i][17] = robot_data.joint_vel_left_shoulder_yaw[i];
            encoderdots[i][18] = robot_data.joint_vel_left_elbow[i];
            encoderdots[i][19] = robot_data.joint_vel_left_wrist_roll[i];
            encoderdots[i][20] = robot_data.joint_vel_left_wrist_pitch[i];
            encoderdots[i][21] = robot_data.joint_vel_left_wrist_yaw[i];
            
            // Right arm velocities (v[22-28])
            encoderdots[i][22] = robot_data.joint_vel_right_shoulder_pitch[i];
            encoderdots[i][23] = robot_data.joint_vel_right_shoulder_roll[i];
            encoderdots[i][24] = robot_data.joint_vel_right_shoulder_yaw[i];
            encoderdots[i][25] = robot_data.joint_vel_right_elbow[i];
            encoderdots[i][26] = robot_data.joint_vel_right_wrist_roll[i];
            encoderdots[i][27] = robot_data.joint_vel_right_wrist_pitch[i];
            encoderdots[i][28] = robot_data.joint_vel_right_wrist_yaw[i];

            motor_torques[i][0] = robot_data.joint_torque_left_hip_pitch[i];
            motor_torques[i][1] = robot_data.joint_torque_left_hip_roll[i];
            motor_torques[i][2] = robot_data.joint_torque_left_hip_yaw[i];
            motor_torques[i][3] = robot_data.joint_torque_left_knee[i];
            motor_torques[i][4] = robot_data.joint_torque_left_ankle_pitch[i];
            motor_torques[i][5] = robot_data.joint_torque_left_ankle_roll[i];
            motor_torques[i][6] = robot_data.joint_torque_right_hip_pitch[i];
            motor_torques[i][7] = robot_data.joint_torque_right_hip_roll[i];
            motor_torques[i][8] = robot_data.joint_torque_right_hip_yaw[i];
            motor_torques[i][9] = robot_data.joint_torque_right_knee[i];
            motor_torques[i][10] = robot_data.joint_torque_right_ankle_pitch[i];
            motor_torques[i][11] = robot_data.joint_torque_right_ankle_roll[i];
            motor_torques[i][12] = robot_data.joint_torque_waist_yaw[i];
            motor_torques[i][13] = robot_data.joint_torque_waist_roll[i];
            motor_torques[i][14] = robot_data.joint_torque_waist_pitch[i];
            motor_torques[i][15] = robot_data.joint_torque_left_shoulder_pitch[i];
            motor_torques[i][16] = robot_data.joint_torque_left_shoulder_roll[i];
            motor_torques[i][17] = robot_data.joint_torque_left_shoulder_yaw[i];
            motor_torques[i][18] = robot_data.joint_torque_left_elbow[i];
            motor_torques[i][19] = robot_data.joint_torque_left_wrist_roll[i];
            motor_torques[i][20] = robot_data.joint_torque_left_wrist_pitch[i];
            motor_torques[i][21] = robot_data.joint_torque_left_wrist_yaw[i];
            motor_torques[i][22] = robot_data.joint_torque_right_shoulder_pitch[i];
            motor_torques[i][23] = robot_data.joint_torque_right_shoulder_roll[i];
            motor_torques[i][24] = robot_data.joint_torque_right_shoulder_yaw[i];
            motor_torques[i][25] = robot_data.joint_torque_right_elbow[i];
            motor_torques[i][26] = robot_data.joint_torque_right_wrist_roll[i];
            motor_torques[i][27] = robot_data.joint_torque_right_wrist_pitch[i];
            motor_torques[i][28] = robot_data.joint_torque_right_wrist_yaw[i];

        }
        
        // Contact states - 2D array [num_rows x num_contacts]
        contact_state.resize(num_rows, std::vector<bool>(num_contacts));
        for (size_t i = 0; i < num_rows; i++) {
            contact_state[i][0] = robot_data.lfoot_contact[i];
            contact_state[i][1] = robot_data.rfoot_contact[i];
        }
    }
};

#endif // TextFileDataStruct_HPP
