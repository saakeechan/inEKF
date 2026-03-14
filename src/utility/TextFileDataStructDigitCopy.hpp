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
#include "Digit/csvParserDigit.hpp"

/**
 * Universal data structure for robot sensor data
 * Contains IMU measurements, joint encoders, encoder velocities, and contact states
 * Can be populated from any robot-specific parser (Digit, G1, Go2, etc.)
 */
class TextFileDataStruct {
public:
    // CSV Parser data source
    DigitDataStruct digit_data;
    
    // Configuration
    size_t num_encoders = 22; // Number of active encoders
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
        digit_data = ParseDigitCSV(filename);
        
        // Direct assignment
        timestamp = digit_data.timestamp;
        gyro_x = digit_data.gyro_x;
        gyro_y = digit_data.gyro_y;
        gyro_z = digit_data.gyro_z;
        acc_x = digit_data.acc_x;
        acc_y = digit_data.acc_y;
        acc_z = digit_data.acc_z;
        pos_x = digit_data.pos_x;
        pos_y = digit_data.pos_y;
        pos_z = digit_data.pos_z;
        vel_x = digit_data.vel_x;
        vel_y = digit_data.vel_y;
        vel_z = digit_data.vel_z;
        quat_w = digit_data.quat_w;
        quat_i = digit_data.quat_i;
        quat_j = digit_data.quat_j;
        quat_k = digit_data.quat_k;
        
        num_rows = digit_data.num_rows;

        
        // Encoders - 2D array [num_rows x num_encoders]
        encoders.resize(num_rows, std::vector<double>(num_encoders));
        encoderdots.resize(num_rows, std::vector<double>(num_encoders));
        
        for (size_t i = 0; i < num_rows; i++) {
            encoders[i][0] = digit_data.left_hip_roll[i];
            encoders[i][1] = digit_data.left_hip_yaw[i];
            encoders[i][2] = digit_data.left_hip_pitch[i];
            encoders[i][3] = digit_data.left_knee[i];
            encoders[i][4] = digit_data.left_shin[i];
            encoders[i][5] = digit_data.left_tarsus[i];
            encoders[i][6] = digit_data.left_heel_spring[i];
            encoders[i][7] = digit_data.left_toe_A[i];
            encoders[i][8] = digit_data.left_toe_B[i];
            encoders[i][9] = digit_data.left_toe_pitch[i];
            encoders[i][10] = digit_data.left_toe_roll[i];
            encoders[i][11] = digit_data.right_hip_roll[i];
            encoders[i][12] = digit_data.right_hip_yaw[i];
            encoders[i][13] = digit_data.right_hip_pitch[i];
            encoders[i][14] = digit_data.right_knee[i];
            encoders[i][15] = digit_data.right_shin[i];
            encoders[i][16] = digit_data.right_tarsus[i];
            encoders[i][17] = digit_data.right_heel_spring[i];
            encoders[i][18] = digit_data.right_toe_A[i];
            encoders[i][19] = digit_data.right_toe_B[i];
            encoders[i][20] = digit_data.right_toe_pitch[i];
            encoders[i][21] = digit_data.right_toe_roll[i];
            
            encoderdots[i][0] = digit_data.left_hip_roll_vel[i];
            encoderdots[i][1] = digit_data.left_hip_yaw_vel[i];
            encoderdots[i][2] = digit_data.left_hip_pitch_vel[i];
            encoderdots[i][3] = digit_data.left_knee_vel[i];
            encoderdots[i][4] = digit_data.left_shin_vel[i];
            encoderdots[i][5] = digit_data.left_tarsus_vel[i];
            encoderdots[i][6] = digit_data.left_heel_spring_vel[i];
            encoderdots[i][7] = digit_data.left_toe_A_vel[i];
            encoderdots[i][8] = digit_data.left_toe_B_vel[i];
            encoderdots[i][9] = digit_data.left_toe_pitch_vel[i];
            encoderdots[i][10] = digit_data.left_toe_roll_vel[i];
            encoderdots[i][11] = digit_data.right_hip_roll_vel[i];
            encoderdots[i][12] = digit_data.right_hip_yaw_vel[i];
            encoderdots[i][13] = digit_data.right_hip_pitch_vel[i];
            encoderdots[i][14] = digit_data.right_knee_vel[i];
            encoderdots[i][15] = digit_data.right_shin_vel[i];
            encoderdots[i][16] = digit_data.right_tarsus_vel[i];
            encoderdots[i][17] = digit_data.right_heel_spring_vel[i];
            encoderdots[i][18] = digit_data.right_toe_A_vel[i];
            encoderdots[i][19] = digit_data.right_toe_B_vel[i];
            encoderdots[i][20] = digit_data.right_toe_pitch_vel[i];
            encoderdots[i][21] = digit_data.right_toe_roll_vel[i];
        }
        
        // Contact states - 2D array [num_rows x num_contacts]
        contact_state.resize(num_rows, std::vector<bool>(num_contacts));
        for (size_t i = 0; i < num_rows; i++) {
            contact_state[i][0] = digit_data.lfoot_contact[i];
            contact_state[i][1] = digit_data.rfoot_contact[i];
        }
    }
};

#endif // TextFileDataStruct_HPP
