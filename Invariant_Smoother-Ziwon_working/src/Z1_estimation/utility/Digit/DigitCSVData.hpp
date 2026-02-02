/**
 * DigitCSVData.hpp
 * 
 * Standalone header file containing the DigitCSVData structure
 * for parsing Digit robot CSV data files.
 * 
 * This header can be included independently without requiring
 * the full InEKF_estimator.hpp dependencies.
 */

#ifndef DIGIT_CSV_DATA_HPP
#define DIGIT_CSV_DATA_HPP

#include <vector>
#include <string>
#include <cstddef>
#include <eigen3/Eigen/Dense>

// Structure to hold parsed Digit robot CSV data
// Matches all 201 columns from digit_cropped_data.csv
struct DigitCSVData {
    std::vector<double> timestamp;
    
    // Base position (columns 1-3)
    std::vector<double> pos_x, pos_y, pos_z;
    
    // Base orientation quaternion (columns 4-7)
    std::vector<double> quat_w, quat_i, quat_j, quat_k;
    
    // Base velocity (columns 8-10)
    std::vector<double> vel_x, vel_y, vel_z;
    
    // Angular rates (columns 11-13)
    std::vector<double> r_rate, p_rate, y_rate;
    
    // Joint positions - Left leg (columns 14-19)
    std::vector<double> left_hip_roll, left_hip_yaw, left_hip_pitch;
    std::vector<double> left_knee, left_toe_A, left_toe_B;
    
    // Joint positions - Left arm (columns 20-23)
    std::vector<double> left_shoulder_roll, left_shoulder_pitch;
    std::vector<double> left_shoulder_yaw, left_elbow;
    
    // Joint positions - Right leg (columns 24-29)
    std::vector<double> right_hip_roll, right_hip_yaw, right_hip_pitch;
    std::vector<double> right_knee, right_toe_A, right_toe_B;
    
    // Joint positions - Right arm (columns 30-33)
    std::vector<double> right_shoulder_roll, right_shoulder_pitch;
    std::vector<double> right_shoulder_yaw, right_elbow;
    
    // Passive joints - Left leg (columns 34-38)
    std::vector<double> left_shin, left_tarsus, left_toe_pitch;
    std::vector<double> left_toe_roll, left_heel_spring;
    
    // Passive joints - Right leg (columns 39-43)
    std::vector<double> right_shin, right_tarsus, right_toe_pitch;
    std::vector<double> right_toe_roll, right_heel_spring;
    
    // Joint velocities - Left leg (columns 44-49)
    std::vector<double> left_hip_roll_vel, left_hip_yaw_vel, left_hip_pitch_vel;
    std::vector<double> left_knee_vel, left_toe_A_vel, left_toe_B_vel;
    
    // Joint velocities - Left arm (columns 50-53)
    std::vector<double> left_shoulder_roll_vel, left_shoulder_pitch_vel;
    std::vector<double> left_shoulder_yaw_vel, left_elbow_vel;
    
    // Joint velocities - Right leg (columns 54-59)
    std::vector<double> right_hip_roll_vel, right_hip_yaw_vel, right_hip_pitch_vel;
    std::vector<double> right_knee_vel, right_toe_A_vel, right_toe_B_vel;
    
    // Joint velocities - Right arm (columns 60-63)
    std::vector<double> right_shoulder_roll_vel, right_shoulder_pitch_vel;
    std::vector<double> right_shoulder_yaw_vel, right_elbow_vel;
    
    // Passive joint velocities - Left leg (columns 64-68)
    std::vector<double> left_shin_vel, left_tarsus_vel, left_toe_pitch_vel;
    std::vector<double> left_toe_roll_vel, left_heel_spring_vel;
    
    // Passive joint velocities - Right leg (columns 69-73)
    std::vector<double> right_shin_vel, right_tarsus_vel, right_toe_pitch_vel;
    std::vector<double> right_toe_roll_vel, right_heel_spring_vel;
    
    // Orientation angles (columns 74-76)
    std::vector<double> roll, pitch, yaw;
    
    // Height (column 77)
    std::vector<double> height;
    
    // Reference velocities - Left leg (columns 78-83)
    std::vector<double> LeftHipRoll_ref_vel, LeftHipYaw_ref_vel, LeftHipPitch_ref_vel;
    std::vector<double> LeftKnee_ref_vel, LeftToeA_ref_vel, LeftToeB_ref_vel;
    
    // Reference velocities - Left arm (columns 84-87)
    std::vector<double> LeftShoulderRoll_ref_vel, LeftShoulderPitch_ref_vel;
    std::vector<double> LeftShoulderYaw_ref_vel, LeftElbow_ref_vel;
    
    // Reference velocities - Right leg (columns 88-93)
    std::vector<double> RightHipRoll_ref_vel, RightHipYaw_ref_vel, RightHipPitch_ref_vel;
    std::vector<double> RightKnee_ref_vel, RightToeA_ref_vel, RightToeB_ref_vel;
    
    // Reference velocities - Right arm (columns 94-97)
    std::vector<double> RightShoulderRoll_ref_vel, RightShoulderPitch_ref_vel;
    std::vector<double> RightShoulderYaw_ref_vel, RightElbow_ref_vel;
    
    // Joint torques - Left leg (columns 98-103)
    std::vector<double> left_hip_roll_tau, left_hip_yaw_tau, left_hip_pitch_tau;
    std::vector<double> left_knee_tau, left_toe_A_tau, left_toe_B_tau;
    
    // Joint torques - Left arm (columns 104-107)
    std::vector<double> left_shoulder_roll_tau, left_shoulder_pitch_tau;
    std::vector<double> left_shoulder_yaw_tau, left_elbow_tau;
    
    // Joint torques - Right leg (columns 108-113)
    std::vector<double> right_hip_roll_tau, right_hip_yaw_tau, right_hip_pitch_tau;
    std::vector<double> right_knee_tau, right_toe_A_tau, right_toe_B_tau;
    
    // Joint torques - Right arm (columns 114-117)
    std::vector<double> right_shoulder_roll_tau, right_shoulder_pitch_tau;
    std::vector<double> right_shoulder_yaw_tau, right_elbow_tau;
    
    // Joint torque commands - Left leg (columns 118-123)
    std::vector<double> left_hip_roll_tau_cmd, left_hip_yaw_tau_cmd, left_hip_pitch_tau_cmd;
    std::vector<double> left_knee_tau_cmd, left_toe_A_tau_cmd, left_toe_B_tau_cmd;
    
    // Joint torque commands - Left arm (columns 124-127)
    std::vector<double> left_shoulder_roll_tau_cmd, left_shoulder_pitch_tau_cmd;
    std::vector<double> left_shoulder_yaw_tau_cmd, left_elbow_tau_cmd;
    
    // Joint torque commands - Right leg (columns 128-133)
    std::vector<double> right_hip_roll_tau_cmd, right_hip_yaw_tau_cmd, right_hip_pitch_tau_cmd;
    std::vector<double> right_knee_tau_cmd, right_toe_A_tau_cmd, right_toe_B_tau_cmd;
    
    // Joint torque commands - Right arm (columns 134-137)
    std::vector<double> right_shoulder_roll_tau_cmd, right_shoulder_pitch_tau_cmd;
    std::vector<double> right_shoulder_yaw_tau_cmd, right_elbow_tau_cmd;
    
    // Position targets - Left leg (columns 138-143)
    std::vector<double> left_hip_roll_pos_tgt, left_hip_yaw_pos_tgt, left_hip_pitch_pos_tgt;
    std::vector<double> left_knee_pos_tgt, left_toe_A_pos_tgt, left_toe_B_pos_tgt;
    
    // Position targets - Right leg (columns 144-149)
    std::vector<double> right_hip_roll_pos_tgt, right_hip_yaw_pos_tgt, right_hip_pitch_pos_tgt;
    std::vector<double> right_knee_pos_tgt, right_toe_A_pos_tgt, right_toe_B_pos_tgt;
    
    // IMU data (columns 150-155)
    std::vector<double> gyro_x, gyro_y, gyro_z;
    std::vector<double> acc_x, acc_y, acc_z;
    
    // Control signals (columns 156-157)
    std::vector<double> turning_direction, cmd;
    
    // Stability array (columns 158-167)
    std::vector<double> stability_0, stability_1, stability_2, stability_3, stability_4;
    std::vector<double> stability_5, stability_6, stability_7, stability_8, stability_9;
    
    // Traction and terminal (columns 168-169)
    std::vector<double> traction, terminal;
    
    // Contact planning and phase (columns 170-174)
    std::vector<double> lfoot_contact_plan, lfoot_contact;
    std::vector<double> rfoot_contact_plan, rfoot_contact;
    std::vector<double> phase_variable;
    
    // Left foot ground reaction forces (columns 175-180)
    std::vector<double> lfoot_grf_x, lfoot_grf_y, lfoot_grf_z;
    std::vector<double> lfoot_grt_x, lfoot_grt_y, lfoot_grt_z;
    
    // Right foot ground reaction forces (columns 181-186)
    std::vector<double> rfoot_grf_x, rfoot_grf_y, rfoot_grf_z;
    std::vector<double> rfoot_grt_x, rfoot_grt_y, rfoot_grt_z;
    
    // Left foot pose (columns 187-193)
    std::vector<double> lfoot_pos_x, lfoot_pos_y, lfoot_pos_z;
    std::vector<double> lfoot_quat_w, lfoot_quat_i, lfoot_quat_j, lfoot_quat_k;
    
    // Right foot pose (columns 194-200)
    std::vector<double> rfoot_pos_x, rfoot_pos_y, rfoot_pos_z;
    std::vector<double> rfoot_quat_w, rfoot_quat_i, rfoot_quat_j, rfoot_quat_k;
    
    // Number of rows loaded
    size_t num_rows = 0;
    
    // Get active encoder positions for a specific row (22 values matching csv_encoder_indices)
    Eigen::VectorXd get_active_encoders(size_t row_idx) const {
        Eigen::VectorXd csv_active_encoders(22);
        csv_active_encoders[0] = left_hip_roll[row_idx];
        csv_active_encoders[1] = left_hip_yaw[row_idx];
        csv_active_encoders[2] = left_hip_pitch[row_idx];
        csv_active_encoders[3] = left_knee[row_idx];
        csv_active_encoders[4] = left_shin[row_idx];
        csv_active_encoders[5] = left_tarsus[row_idx];
        csv_active_encoders[6] = left_heel_spring[row_idx];
        csv_active_encoders[7] = left_toe_A[row_idx];
        csv_active_encoders[8] = left_toe_B[row_idx];
        csv_active_encoders[9] = left_toe_pitch[row_idx];
        csv_active_encoders[10] = left_toe_roll[row_idx];
        csv_active_encoders[11] = right_hip_roll[row_idx];
        csv_active_encoders[12] = right_hip_yaw[row_idx];
        csv_active_encoders[13] = right_hip_pitch[row_idx];
        csv_active_encoders[14] = right_knee[row_idx];
        csv_active_encoders[15] = right_shin[row_idx];
        csv_active_encoders[16] = right_tarsus[row_idx];
        csv_active_encoders[17] = right_heel_spring[row_idx];
        csv_active_encoders[18] = right_toe_A[row_idx];
        csv_active_encoders[19] = right_toe_B[row_idx];
        csv_active_encoders[20] = right_toe_pitch[row_idx];
        csv_active_encoders[21] = right_toe_roll[row_idx];
        return csv_active_encoders;
    }
    
    // Get active encoder velocities for a specific row (22 values matching csv_encoderdot_indices)
    Eigen::VectorXd get_active_encoderdots(size_t row_idx) const {
        Eigen::VectorXd csv_active_encoderdots(22);
        csv_active_encoderdots[0] = left_hip_roll_vel[row_idx];
        csv_active_encoderdots[1] = left_hip_yaw_vel[row_idx];
        csv_active_encoderdots[2] = left_hip_pitch_vel[row_idx];
        csv_active_encoderdots[3] = left_knee_vel[row_idx];
        csv_active_encoderdots[4] = left_shin_vel[row_idx];
        csv_active_encoderdots[5] = left_tarsus_vel[row_idx];
        csv_active_encoderdots[6] = left_heel_spring_vel[row_idx];
        csv_active_encoderdots[7] = left_toe_A_vel[row_idx];
        csv_active_encoderdots[8] = left_toe_B_vel[row_idx];
        csv_active_encoderdots[9] = left_toe_pitch_vel[row_idx];
        csv_active_encoderdots[10] = left_toe_roll_vel[row_idx];
        csv_active_encoderdots[11] = right_hip_roll_vel[row_idx];
        csv_active_encoderdots[12] = right_hip_yaw_vel[row_idx];
        csv_active_encoderdots[13] = right_hip_pitch_vel[row_idx];
        csv_active_encoderdots[14] = right_knee_vel[row_idx];
        csv_active_encoderdots[15] = right_shin_vel[row_idx];
        csv_active_encoderdots[16] = right_tarsus_vel[row_idx];
        csv_active_encoderdots[17] = right_heel_spring_vel[row_idx];
        csv_active_encoderdots[18] = right_toe_A_vel[row_idx];
        csv_active_encoderdots[19] = right_toe_B_vel[row_idx];
        csv_active_encoderdots[20] = right_toe_pitch_vel[row_idx];
        csv_active_encoderdots[21] = right_toe_roll_vel[row_idx];
        return csv_active_encoderdots;
    }
    
    // Get contact state for a specific row (2 values: left foot, right foot)
    std::vector<bool> get_contact_state(size_t row_idx) const {
        std::vector<bool> contact_state(2);
        // Contact data is binary: 1.0 = contact, 0.0 = no contact
        contact_state[0] = (lfoot_contact[row_idx]);  // Left foot contact
        contact_state[1] = (rfoot_contact[row_idx]);  // Right foot contact
        return contact_state;
    }
};

// Function declaration for the CSV parser
DigitCSVData ParseDigitCSV(const std::string& csv_filename);

#endif // DIGIT_CSV_DATA_HPP
