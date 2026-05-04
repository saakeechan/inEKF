#ifndef CSV_PARSER_HPP
#define CSV_PARSER_HPP

#include <string>
#include <vector>
#include <cstddef>

// Structure to hold parsed G1 robot CSV data
// Matches all 238 columns from G1LoggingData.csv
struct DataStruct {
    // Counter and timestamp (columns 0-1)
    std::vector<double> count;
    std::vector<double> timestamp;
    
    // Base position (columns 2-4)
    std::vector<double> pos_x, pos_y, pos_z;
    
    // Base orientation quaternion (columns 5-8)
    std::vector<double> quat_w, quat_i, quat_j, quat_k;
    
    // Orientation angles (columns 9-11)
    std::vector<double> roll, pitch, yaw;
    
    // Base velocity (columns 12-14)
    std::vector<double> vel_x, vel_y, vel_z;
    
    // Angular velocity (columns 15-17)
    std::vector<double> omega_x, omega_y, omega_z;
    
    // Linear acceleration (columns 18-20)
    std::vector<double> acc_x, acc_y, acc_z;
    
    // Angular acceleration (columns 21-23)
    std::vector<double> alpha_x, alpha_y, alpha_z;
    
    // Contact planning and phase (columns 24-28)
    std::vector<double> lfoot_contact_plan, lfoot_contact;
    std::vector<double> rfoot_contact_plan, rfoot_contact;
    std::vector<double> phase_variable;
    
    // Left foot ground reaction forces (columns 29-34)
    std::vector<double> lfoot_grf_x, lfoot_grf_y, lfoot_grf_z;
    std::vector<double> lfoot_grt_x, lfoot_grt_y, lfoot_grt_z;
    
    // Right foot ground reaction forces (columns 35-40)
    std::vector<double> rfoot_grf_x, rfoot_grf_y, rfoot_grf_z;
    std::vector<double> rfoot_grt_x, rfoot_grt_y, rfoot_grt_z;
    
    // Left foot pose (columns 41-47)
    std::vector<double> lfoot_pos_x, lfoot_pos_y, lfoot_pos_z;
    std::vector<double> lfoot_quat_w, lfoot_quat_i, lfoot_quat_j, lfoot_quat_k;
    
    // Right foot pose (columns 48-54)
    std::vector<double> rfoot_pos_x, rfoot_pos_y, rfoot_pos_z;
    std::vector<double> rfoot_quat_w, rfoot_quat_i, rfoot_quat_j, rfoot_quat_k;
    
    // Body frame IMU data (columns 55-60)
    std::vector<double> acc_body_x, acc_body_y, acc_body_z;
    std::vector<double> gyro_body_x, gyro_body_y, gyro_body_z;
    
    // Forward kinematics foot positions
    std::vector<double> fk_left_foot_pos_x, fk_left_foot_pos_y, fk_left_foot_pos_z;
    std::vector<double> fk_right_foot_pos_x, fk_right_foot_pos_y, fk_right_foot_pos_z;
    
    // Forward kinematics foot velocities
    std::vector<double> fk_left_foot_vel_x, fk_left_foot_vel_y, fk_left_foot_vel_z;
    std::vector<double> fk_right_foot_vel_x, fk_right_foot_vel_y, fk_right_foot_vel_z;
    
    // Joint positions (columns 61-103) - 43 joints total
    std::vector<double> joint_pos_left_hip_pitch, joint_pos_right_hip_pitch, joint_pos_waist_yaw;
    std::vector<double> joint_pos_left_hip_roll, joint_pos_right_hip_roll, joint_pos_waist_roll;
    std::vector<double> joint_pos_left_hip_yaw, joint_pos_right_hip_yaw, joint_pos_waist_pitch;
    std::vector<double> joint_pos_left_knee, joint_pos_right_knee;
    std::vector<double> joint_pos_left_shoulder_pitch, joint_pos_right_shoulder_pitch;
    std::vector<double> joint_pos_left_ankle_pitch, joint_pos_right_ankle_pitch;
    std::vector<double> joint_pos_left_shoulder_roll, joint_pos_right_shoulder_roll;
    std::vector<double> joint_pos_left_ankle_roll, joint_pos_right_ankle_roll;
    std::vector<double> joint_pos_left_shoulder_yaw, joint_pos_right_shoulder_yaw;
    std::vector<double> joint_pos_left_elbow, joint_pos_right_elbow;
    std::vector<double> joint_pos_left_wrist_roll, joint_pos_right_wrist_roll;
    std::vector<double> joint_pos_left_wrist_pitch, joint_pos_right_wrist_pitch;
    std::vector<double> joint_pos_left_wrist_yaw, joint_pos_right_wrist_yaw;
    std::vector<double> joint_pos_left_hand_index_0, joint_pos_left_hand_middle_0, joint_pos_left_hand_thumb_0;
    std::vector<double> joint_pos_right_hand_index_0, joint_pos_right_hand_middle_0, joint_pos_right_hand_thumb_0;
    std::vector<double> joint_pos_left_hand_index_1, joint_pos_left_hand_middle_1, joint_pos_left_hand_thumb_1;
    std::vector<double> joint_pos_right_hand_index_1, joint_pos_right_hand_middle_1, joint_pos_right_hand_thumb_1;
    std::vector<double> joint_pos_left_hand_thumb_2, joint_pos_right_hand_thumb_2;
    
    // Joint velocities (columns 104-146) - 43 joints total
    std::vector<double> joint_vel_left_hip_pitch, joint_vel_right_hip_pitch, joint_vel_waist_yaw;
    std::vector<double> joint_vel_left_hip_roll, joint_vel_right_hip_roll, joint_vel_waist_roll;
    std::vector<double> joint_vel_left_hip_yaw, joint_vel_right_hip_yaw, joint_vel_waist_pitch;
    std::vector<double> joint_vel_left_knee, joint_vel_right_knee;
    std::vector<double> joint_vel_left_shoulder_pitch, joint_vel_right_shoulder_pitch;
    std::vector<double> joint_vel_left_ankle_pitch, joint_vel_right_ankle_pitch;
    std::vector<double> joint_vel_left_shoulder_roll, joint_vel_right_shoulder_roll;
    std::vector<double> joint_vel_left_ankle_roll, joint_vel_right_ankle_roll;
    std::vector<double> joint_vel_left_shoulder_yaw, joint_vel_right_shoulder_yaw;
    std::vector<double> joint_vel_left_elbow, joint_vel_right_elbow;
    std::vector<double> joint_vel_left_wrist_roll, joint_vel_right_wrist_roll;
    std::vector<double> joint_vel_left_wrist_pitch, joint_vel_right_wrist_pitch;
    std::vector<double> joint_vel_left_wrist_yaw, joint_vel_right_wrist_yaw;
    std::vector<double> joint_vel_left_hand_index_0, joint_vel_left_hand_middle_0, joint_vel_left_hand_thumb_0;
    std::vector<double> joint_vel_right_hand_index_0, joint_vel_right_hand_middle_0, joint_vel_right_hand_thumb_0;
    std::vector<double> joint_vel_left_hand_index_1, joint_vel_left_hand_middle_1, joint_vel_left_hand_thumb_1;
    std::vector<double> joint_vel_right_hand_index_1, joint_vel_right_hand_middle_1, joint_vel_right_hand_thumb_1;
    std::vector<double> joint_vel_left_hand_thumb_2, joint_vel_right_hand_thumb_2;
    
    // Joint torques (columns 147-189) - 43 joints total
    std::vector<double> joint_torque_left_hip_pitch, joint_torque_right_hip_pitch, joint_torque_waist_yaw;
    std::vector<double> joint_torque_left_hip_roll, joint_torque_right_hip_roll, joint_torque_waist_roll;
    std::vector<double> joint_torque_left_hip_yaw, joint_torque_right_hip_yaw, joint_torque_waist_pitch;
    std::vector<double> joint_torque_left_knee, joint_torque_right_knee;
    std::vector<double> joint_torque_left_shoulder_pitch, joint_torque_right_shoulder_pitch;
    std::vector<double> joint_torque_left_ankle_pitch, joint_torque_right_ankle_pitch;
    std::vector<double> joint_torque_left_shoulder_roll, joint_torque_right_shoulder_roll;
    std::vector<double> joint_torque_left_ankle_roll, joint_torque_right_ankle_roll;
    std::vector<double> joint_torque_left_shoulder_yaw, joint_torque_right_shoulder_yaw;
    std::vector<double> joint_torque_left_elbow, joint_torque_right_elbow;
    std::vector<double> joint_torque_left_wrist_roll, joint_torque_right_wrist_roll;
    std::vector<double> joint_torque_left_wrist_pitch, joint_torque_right_wrist_pitch;
    std::vector<double> joint_torque_left_wrist_yaw, joint_torque_right_wrist_yaw;
    std::vector<double> joint_torque_left_hand_index_0, joint_torque_left_hand_middle_0, joint_torque_left_hand_thumb_0;
    std::vector<double> joint_torque_right_hand_index_0, joint_torque_right_hand_middle_0, joint_torque_right_hand_thumb_0;
    std::vector<double> joint_torque_left_hand_index_1, joint_torque_left_hand_middle_1, joint_torque_left_hand_thumb_1;
    std::vector<double> joint_torque_right_hand_index_1, joint_torque_right_hand_middle_1, joint_torque_right_hand_thumb_1;
    std::vector<double> joint_torque_left_hand_thumb_2, joint_torque_right_hand_thumb_2;
    
    // Joint actions (columns 190-232) - 43 joints total
    std::vector<double> joint_action_left_hip_pitch, joint_action_right_hip_pitch, joint_action_waist_yaw;
    std::vector<double> joint_action_left_hip_roll, joint_action_right_hip_roll, joint_action_waist_roll;
    std::vector<double> joint_action_left_hip_yaw, joint_action_right_hip_yaw, joint_action_waist_pitch;
    std::vector<double> joint_action_left_knee, joint_action_right_knee;
    std::vector<double> joint_action_left_shoulder_pitch, joint_action_right_shoulder_pitch;
    std::vector<double> joint_action_left_ankle_pitch, joint_action_right_ankle_pitch;
    std::vector<double> joint_action_left_shoulder_roll, joint_action_right_shoulder_roll;
    std::vector<double> joint_action_left_ankle_roll, joint_action_right_ankle_roll;
    std::vector<double> joint_action_left_shoulder_yaw, joint_action_right_shoulder_yaw;
    std::vector<double> joint_action_left_elbow, joint_action_right_elbow;
    std::vector<double> joint_action_left_wrist_roll, joint_action_right_wrist_roll;
    std::vector<double> joint_action_left_wrist_pitch, joint_action_right_wrist_pitch;
    std::vector<double> joint_action_left_wrist_yaw, joint_action_right_wrist_yaw;
    std::vector<double> joint_action_left_hand_index_0, joint_action_left_hand_middle_0, joint_action_left_hand_thumb_0;
    std::vector<double> joint_action_right_hand_index_0, joint_action_right_hand_middle_0, joint_action_right_hand_thumb_0;
    std::vector<double> joint_action_left_hand_index_1, joint_action_left_hand_middle_1, joint_action_left_hand_thumb_1;
    std::vector<double> joint_action_right_hand_index_1, joint_action_right_hand_middle_1, joint_action_right_hand_thumb_1;
    std::vector<double> joint_action_left_hand_thumb_2, joint_action_right_hand_thumb_2;
    
    // Command velocities and stability (columns 233-237)
    std::vector<double> cmd_vel_x, cmd_vel_y, cmd_vel_z;
    std::vector<double> cmd_w;
    std::vector<double> stability;
    
    // Number of rows loaded
    size_t num_rows = 0;
    
    // Configuration
    size_t num_encoders = 29; // Number of active encoders
    size_t num_contacts = 2;  // Number of contact points
    
    // Reorganized 2D arrays for easier access
    // Joint encoder positions - 2D array [num_rows x num_encoders]
    std::vector<std::vector<double>> encoders;
    
    // Joint encoder velocities - 2D array [num_rows x num_encoders]
    std::vector<std::vector<double>> encoderdots;
    
    // Motor torques - 2D array [num_rows x num_motors]
    std::vector<std::vector<double>> motor_torques;
    
    // Motor torque commands - 2D array [num_rows x num_motors]
    std::vector<std::vector<double>> motor_torques_cmd;
    
    // Command velocity - 2D array [num_rows x 3] (x, y, z)
    std::vector<std::vector<double>> CMD_V;
    
    // Contact states - 2D array [num_rows x num_contacts]
    std::vector<std::vector<bool>> contact_state;
};

/**
 * Parse CSV file containing G1 robot sensor data
 * @param csv_filename Path to the CSV file
 * @return DataStruct structure containing parsed data
 */
DataStruct ParseCSV(const std::string& csv_filename);

#endif // CSV_PARSER_G1_HPP
