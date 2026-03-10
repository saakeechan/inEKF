/**
 * CSV Parser for Digit Robot Data
 * 
 * This file contains the implementation of ParseDigitCSV function
 * which parses all 201 columns from digit_cropped_data.csv
 * 
 * See CSV_PARSER_USAGE.md for detailed usage documentation
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>
#include "csvParserDigit.hpp"

using namespace std;

DigitDataStruct ParseDigitCSV(const std::string& csv_filename) {
    
    DigitDataStruct data;
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
    
    // Skip the header line
    std::string header_line;
    std::getline(file, header_line);
    
    // Parse each data row
    std::string line;
    int row_count = 0;
    
    while (std::getline(file, line)) {
        std::vector<double> row_values;
        std::stringstream ss(line);
        std::string value;
        
        // Parse all comma-separated values in this row
        while (std::getline(ss, value, ',')) {
            // Skip empty values (from trailing commas)
            if (value.empty()) {
                continue;
            }
            try {
                row_values.push_back(std::stod(value));
            } catch (const std::exception& e) {
                std::cerr << "Error parsing value '" << value << "' at row " << row_count << std::endl;
                row_values.push_back(0.0); // Default to 0 on parse error
            }
        }
        
        // // Ensure we have enough columns (should be 201: timestamp + 200 data columns)
        // if (row_values.size() < 201) {
        //     std::cerr << "Warning: Row " << row_count << " has only " << row_values.size() 
        //               << " columns (expected 201)" << std::endl;
        // }
        
        // Assign values to named variables (column indexing starts at 0)
        if (row_values.size() >= 1) data.timestamp.push_back(row_values[0]);
        
        // Base position (1-3)
        if (row_values.size() >= 4) {
            data.pos_x.push_back(row_values[1]);
            data.pos_y.push_back(row_values[2]);
            data.pos_z.push_back(row_values[3]);
        }
        
        // Base orientation quaternion (4-7)
        if (row_values.size() >= 8) {
            data.quat_w.push_back(row_values[4]);
            data.quat_i.push_back(row_values[5]);
            data.quat_j.push_back(row_values[6]);
            data.quat_k.push_back(row_values[7]);
        }
        
        // Base velocity (8-10)
        if (row_values.size() >= 11) {
            data.vel_x.push_back(row_values[8]);
            data.vel_y.push_back(row_values[9]);
            data.vel_z.push_back(row_values[10]);
        }
        
        // Angular rates (11-13)
        if (row_values.size() >= 14) {
            data.r_rate.push_back(row_values[11]);
            data.p_rate.push_back(row_values[12]);
            data.y_rate.push_back(row_values[13]);
        }
        
        // Joint positions - Left leg (14-19)
        if (row_values.size() >= 20) {
            data.left_hip_roll.push_back(row_values[14]);
            data.left_hip_yaw.push_back(row_values[15]);
            data.left_hip_pitch.push_back(row_values[16]);
            data.left_knee.push_back(row_values[17]);
            data.left_toe_A.push_back(row_values[18]);
            data.left_toe_B.push_back(row_values[19]);
        }
        
        // Joint positions - Left arm (20-23)
        if (row_values.size() >= 24) {
            data.left_shoulder_roll.push_back(row_values[20]);
            data.left_shoulder_pitch.push_back(row_values[21]);
            data.left_shoulder_yaw.push_back(row_values[22]);
            data.left_elbow.push_back(row_values[23]);
        }
        
        // Joint positions - Right leg (24-29)
        if (row_values.size() >= 30) {
            data.right_hip_roll.push_back(row_values[24]);
            data.right_hip_yaw.push_back(row_values[25]);
            data.right_hip_pitch.push_back(row_values[26]);
            data.right_knee.push_back(row_values[27]);
            data.right_toe_A.push_back(row_values[28]);
            data.right_toe_B.push_back(row_values[29]);
        }
        
        // Joint positions - Right arm (30-33)
        if (row_values.size() >= 34) {
            data.right_shoulder_roll.push_back(row_values[30]);
            data.right_shoulder_pitch.push_back(row_values[31]);
            data.right_shoulder_yaw.push_back(row_values[32]);
            data.right_elbow.push_back(row_values[33]);
        }
        
        // Passive joints - Left leg (34-38)
        if (row_values.size() >= 39) {
            data.left_shin.push_back(row_values[34]);
            data.left_tarsus.push_back(row_values[35]);
            data.left_toe_pitch.push_back(row_values[36]);
            data.left_toe_roll.push_back(row_values[37]);
            data.left_heel_spring.push_back(row_values[38]);
        }
        
        // Passive joints - Right leg (39-43)
        if (row_values.size() >= 44) {
            data.right_shin.push_back(row_values[39]);
            data.right_tarsus.push_back(row_values[40]);
            data.right_toe_pitch.push_back(row_values[41]);
            data.right_toe_roll.push_back(row_values[42]);
            data.right_heel_spring.push_back(row_values[43]);
        }
        
        // Joint velocities - Left leg (44-49)
        if (row_values.size() >= 50) {
            data.left_hip_roll_vel.push_back(row_values[44]);
            data.left_hip_yaw_vel.push_back(row_values[45]);
            data.left_hip_pitch_vel.push_back(row_values[46]);
            data.left_knee_vel.push_back(row_values[47]);
            data.left_toe_A_vel.push_back(row_values[48]);
            data.left_toe_B_vel.push_back(row_values[49]);
        }
        
        // Joint velocities - Left arm (50-53)
        if (row_values.size() >= 54) {
            data.left_shoulder_roll_vel.push_back(row_values[50]);
            data.left_shoulder_pitch_vel.push_back(row_values[51]);
            data.left_shoulder_yaw_vel.push_back(row_values[52]);
            data.left_elbow_vel.push_back(row_values[53]);
        }
        
        // Joint velocities - Right leg (54-59)
        if (row_values.size() >= 60) {
            data.right_hip_roll_vel.push_back(row_values[54]);
            data.right_hip_yaw_vel.push_back(row_values[55]);
            data.right_hip_pitch_vel.push_back(row_values[56]);
            data.right_knee_vel.push_back(row_values[57]);
            data.right_toe_A_vel.push_back(row_values[58]);
            data.right_toe_B_vel.push_back(row_values[59]);
        }
        
        // Joint velocities - Right arm (60-63)
        if (row_values.size() >= 64) {
            data.right_shoulder_roll_vel.push_back(row_values[60]);
            data.right_shoulder_pitch_vel.push_back(row_values[61]);
            data.right_shoulder_yaw_vel.push_back(row_values[62]);
            data.right_elbow_vel.push_back(row_values[63]);
        }
        
        // Passive joint velocities - Left leg (64-68)
        if (row_values.size() >= 69) {
            data.left_shin_vel.push_back(row_values[64]);
            data.left_tarsus_vel.push_back(row_values[65]);
            data.left_toe_pitch_vel.push_back(row_values[66]);
            data.left_toe_roll_vel.push_back(row_values[67]);
            data.left_heel_spring_vel.push_back(row_values[68]);
        }
        
        // Passive joint velocities - Right leg (69-73)
        if (row_values.size() >= 74) {
            data.right_shin_vel.push_back(row_values[69]);
            data.right_tarsus_vel.push_back(row_values[70]);
            data.right_toe_pitch_vel.push_back(row_values[71]);
            data.right_toe_roll_vel.push_back(row_values[72]);
            data.right_heel_spring_vel.push_back(row_values[73]);
        }
        
        // Orientation angles (74-76)
        if (row_values.size() >= 77) {
            data.roll.push_back(row_values[74]);
            data.pitch.push_back(row_values[75]);
            data.yaw.push_back(row_values[76]);
        }
        
        // Height (77)
        if (row_values.size() >= 78) {
            data.height.push_back(row_values[77]);
        }
        
        // Reference velocities - Left leg (78-83)
        if (row_values.size() >= 84) {
            data.LeftHipRoll_ref_vel.push_back(row_values[78]);
            data.LeftHipYaw_ref_vel.push_back(row_values[79]);
            data.LeftHipPitch_ref_vel.push_back(row_values[80]);
            data.LeftKnee_ref_vel.push_back(row_values[81]);
            data.LeftToeA_ref_vel.push_back(row_values[82]);
            data.LeftToeB_ref_vel.push_back(row_values[83]);
        }
        
        // Reference velocities - Left arm (84-87)
        if (row_values.size() >= 88) {
            data.LeftShoulderRoll_ref_vel.push_back(row_values[84]);
            data.LeftShoulderPitch_ref_vel.push_back(row_values[85]);
            data.LeftShoulderYaw_ref_vel.push_back(row_values[86]);
            data.LeftElbow_ref_vel.push_back(row_values[87]);
        }
        
        // Reference velocities - Right leg (88-93)
        if (row_values.size() >= 94) {
            data.RightHipRoll_ref_vel.push_back(row_values[88]);
            data.RightHipYaw_ref_vel.push_back(row_values[89]);
            data.RightHipPitch_ref_vel.push_back(row_values[90]);
            data.RightKnee_ref_vel.push_back(row_values[91]);
            data.RightToeA_ref_vel.push_back(row_values[92]);
            data.RightToeB_ref_vel.push_back(row_values[93]);
        }
        
        // Reference velocities - Right arm (94-97)
        if (row_values.size() >= 98) {
            data.RightShoulderRoll_ref_vel.push_back(row_values[94]);
            data.RightShoulderPitch_ref_vel.push_back(row_values[95]);
            data.RightShoulderYaw_ref_vel.push_back(row_values[96]);
            data.RightElbow_ref_vel.push_back(row_values[97]);
        }
        
        // Joint torques - Left leg (98-103)
        if (row_values.size() >= 104) {
            data.left_hip_roll_tau.push_back(row_values[98]);
            data.left_hip_yaw_tau.push_back(row_values[99]);
            data.left_hip_pitch_tau.push_back(row_values[100]);
            data.left_knee_tau.push_back(row_values[101]);
            data.left_toe_A_tau.push_back(row_values[102]);
            data.left_toe_B_tau.push_back(row_values[103]);
        }
        
        // Joint torques - Left arm (104-107)
        if (row_values.size() >= 108) {
            data.left_shoulder_roll_tau.push_back(row_values[104]);
            data.left_shoulder_pitch_tau.push_back(row_values[105]);
            data.left_shoulder_yaw_tau.push_back(row_values[106]);
            data.left_elbow_tau.push_back(row_values[107]);
        }
        
        // Joint torques - Right leg (108-113)
        if (row_values.size() >= 114) {
            data.right_hip_roll_tau.push_back(row_values[108]);
            data.right_hip_yaw_tau.push_back(row_values[109]);
            data.right_hip_pitch_tau.push_back(row_values[110]);
            data.right_knee_tau.push_back(row_values[111]);
            data.right_toe_A_tau.push_back(row_values[112]);
            data.right_toe_B_tau.push_back(row_values[113]);
        }
        
        // Joint torques - Right arm (114-117)
        if (row_values.size() >= 118) {
            data.right_shoulder_roll_tau.push_back(row_values[114]);
            data.right_shoulder_pitch_tau.push_back(row_values[115]);
            data.right_shoulder_yaw_tau.push_back(row_values[116]);
            data.right_elbow_tau.push_back(row_values[117]);
        }
        
        // Joint torque commands - Left leg (118-123)
        if (row_values.size() >= 124) {
            data.left_hip_roll_tau_cmd.push_back(row_values[118]);
            data.left_hip_yaw_tau_cmd.push_back(row_values[119]);
            data.left_hip_pitch_tau_cmd.push_back(row_values[120]);
            data.left_knee_tau_cmd.push_back(row_values[121]);
            data.left_toe_A_tau_cmd.push_back(row_values[122]);
            data.left_toe_B_tau_cmd.push_back(row_values[123]);
        }
        
        // Joint torque commands - Left arm (124-127)
        if (row_values.size() >= 128) {
            data.left_shoulder_roll_tau_cmd.push_back(row_values[124]);
            data.left_shoulder_pitch_tau_cmd.push_back(row_values[125]);
            data.left_shoulder_yaw_tau_cmd.push_back(row_values[126]);
            data.left_elbow_tau_cmd.push_back(row_values[127]);
        }
        
        // Joint torque commands - Right leg (128-133)
        if (row_values.size() >= 134) {
            data.right_hip_roll_tau_cmd.push_back(row_values[128]);
            data.right_hip_yaw_tau_cmd.push_back(row_values[129]);
            data.right_hip_pitch_tau_cmd.push_back(row_values[130]);
            data.right_knee_tau_cmd.push_back(row_values[131]);
            data.right_toe_A_tau_cmd.push_back(row_values[132]);
            data.right_toe_B_tau_cmd.push_back(row_values[133]);
        }
        
        // Joint torque commands - Right arm (134-137)
        if (row_values.size() >= 138) {
            data.right_shoulder_roll_tau_cmd.push_back(row_values[134]);
            data.right_shoulder_pitch_tau_cmd.push_back(row_values[135]);
            data.right_shoulder_yaw_tau_cmd.push_back(row_values[136]);
            data.right_elbow_tau_cmd.push_back(row_values[137]);
        }
        
        // Position targets - Left leg (138-143)
        if (row_values.size() >= 144) {
            data.left_hip_roll_pos_tgt.push_back(row_values[138]);
            data.left_hip_yaw_pos_tgt.push_back(row_values[139]);
            data.left_hip_pitch_pos_tgt.push_back(row_values[140]);
            data.left_knee_pos_tgt.push_back(row_values[141]);
            data.left_toe_A_pos_tgt.push_back(row_values[142]);
            data.left_toe_B_pos_tgt.push_back(row_values[143]);
        }
        
        // Position targets - Right leg (144-149)
        if (row_values.size() >= 150) {
            data.right_hip_roll_pos_tgt.push_back(row_values[144]);
            data.right_hip_yaw_pos_tgt.push_back(row_values[145]);
            data.right_hip_pitch_pos_tgt.push_back(row_values[146]);
            data.right_knee_pos_tgt.push_back(row_values[147]);
            data.right_toe_A_pos_tgt.push_back(row_values[148]);
            data.right_toe_B_pos_tgt.push_back(row_values[149]);
        }
        
        // IMU data (150-155)
        if (row_values.size() >= 156) {
            data.gyro_x.push_back(row_values[150]);
            data.gyro_y.push_back(row_values[151]);
            data.gyro_z.push_back(row_values[152]);
            data.acc_x.push_back(row_values[153]);
            data.acc_y.push_back(row_values[154]);
            data.acc_z.push_back(row_values[155]);
        }
        
        // Control signals (156-157)
        if (row_values.size() >= 158) {
            data.turning_direction.push_back(row_values[156]);
            data.cmd.push_back(row_values[157]);
        }
        
        // Stability array (158-167)
        if (row_values.size() >= 168) {
            data.stability_0.push_back(row_values[158]);
            data.stability_1.push_back(row_values[159]);
            data.stability_2.push_back(row_values[160]);
            data.stability_3.push_back(row_values[161]);
            data.stability_4.push_back(row_values[162]);
            data.stability_5.push_back(row_values[163]);
            data.stability_6.push_back(row_values[164]);
            data.stability_7.push_back(row_values[165]);
            data.stability_8.push_back(row_values[166]);
            data.stability_9.push_back(row_values[167]);
        }
        
        // Traction and terminal (168-169)
        if (row_values.size() >= 170) {
            data.traction.push_back(row_values[168]);
            data.terminal.push_back(row_values[169]);
        }
        
        // Contact planning and phase (170-174)
        if (row_values.size() >= 175) {
            data.lfoot_contact_plan.push_back(row_values[170]);
            data.lfoot_contact.push_back(row_values[171]);
            data.rfoot_contact_plan.push_back(row_values[172]);
            data.rfoot_contact.push_back(row_values[173]);
            data.phase_variable.push_back(row_values[174]);
        }
        
        // Left foot ground reaction forces (175-180)
        if (row_values.size() >= 181) {
            data.lfoot_grf_x.push_back(row_values[175]);
            data.lfoot_grf_y.push_back(row_values[176]);
            data.lfoot_grf_z.push_back(row_values[177]);
            data.lfoot_grt_x.push_back(row_values[178]);
            data.lfoot_grt_y.push_back(row_values[179]);
            data.lfoot_grt_z.push_back(row_values[180]);
        }
        
        // Right foot ground reaction forces (181-186)
        if (row_values.size() >= 187) {
            data.rfoot_grf_x.push_back(row_values[181]);
            data.rfoot_grf_y.push_back(row_values[182]);
            data.rfoot_grf_z.push_back(row_values[183]);
            data.rfoot_grt_x.push_back(row_values[184]);
            data.rfoot_grt_y.push_back(row_values[185]);
            data.rfoot_grt_z.push_back(row_values[186]);
        }
        
        // Left foot pose (187-193)
        if (row_values.size() >= 194) {
            data.lfoot_pos_x.push_back(row_values[187]);
            data.lfoot_pos_y.push_back(row_values[188]);
            data.lfoot_pos_z.push_back(row_values[189]);
            data.lfoot_quat_w.push_back(row_values[190]);
            data.lfoot_quat_i.push_back(row_values[191]);
            data.lfoot_quat_j.push_back(row_values[192]);
            data.lfoot_quat_k.push_back(row_values[193]);
        }
        
        // Right foot pose (194-200)
        if (row_values.size() >= 201) {
            data.rfoot_pos_x.push_back(row_values[194]);
            data.rfoot_pos_y.push_back(row_values[195]);
            data.rfoot_pos_z.push_back(row_values[196]);
            data.rfoot_quat_w.push_back(row_values[197]);
            data.rfoot_quat_i.push_back(row_values[198]);
            data.rfoot_quat_j.push_back(row_values[199]);
            data.rfoot_quat_k.push_back(row_values[200]);
        }
        
        row_count++;
    }
    
    file.close();
    data.num_rows = row_count;
    
    std::cout << "Total columns parsed: 201 (timestamp + 200 data columns)" << std::endl;
    
    return data;
}