/**
 * CSV Parser for G1 Robot Data
 * 
 * This file contains the implementation of ParseG1CSV function
 * which parses all 238 columns from G1LoggingData.csv
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdlib>
#include "csvParserG1_FootPosVel.hpp"

using namespace std;

G1DataStruct ParseG1CSV(const std::string& csv_filename) {
    
    G1DataStruct data;
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
            try {
                row_values.push_back(std::stod(value));
            } catch (const std::exception& e) {
                std::cerr << "Warning: Could not parse value '" << value << "' at row " << row_count << std::endl;
                row_values.push_back(0.0);
            }
        }
        
        // Assign values to named variables (column indexing starts at 0)
        if (row_values.size() >= 1) data.count.push_back(row_values[0]);
        if (row_values.size() >= 2) data.timestamp.push_back(row_values[1]);
        
        // Base position (2-4)
        if (row_values.size() >= 5) {
            data.pos_x.push_back(row_values[2]);
            data.pos_y.push_back(row_values[3]);
            data.pos_z.push_back(row_values[4]);
        }
        
        // Base orientation quaternion (5-8)
        if (row_values.size() >= 9) {
            data.quat_w.push_back(row_values[5]);
            data.quat_i.push_back(row_values[6]);
            data.quat_j.push_back(row_values[7]);
            data.quat_k.push_back(row_values[8]);
        }
        
        // Orientation angles (9-11)
        if (row_values.size() >= 12) {
            data.roll.push_back(row_values[9]);
            data.pitch.push_back(row_values[10]);
            data.yaw.push_back(row_values[11]);
        }
        
        // Base velocity (12-14)
        if (row_values.size() >= 15) {
            data.vel_x.push_back(row_values[12]);
            data.vel_y.push_back(row_values[13]);
            data.vel_z.push_back(row_values[14]);
        }
        
        // Angular velocity (15-17)
        if (row_values.size() >= 18) {
            data.omega_x.push_back(row_values[15]);
            data.omega_y.push_back(row_values[16]);
            data.omega_z.push_back(row_values[17]);
        }
        
        // Linear acceleration (18-20)
        if (row_values.size() >= 21) {
            data.acc_x.push_back(row_values[18]);
            data.acc_y.push_back(row_values[19]);
            data.acc_z.push_back(row_values[20]);
        }
        
        // Angular acceleration (21-23)
        if (row_values.size() >= 24) {
            data.alpha_x.push_back(row_values[21]);
            data.alpha_y.push_back(row_values[22]);
            data.alpha_z.push_back(row_values[23]);
        }
        
        // Body frame IMU data (24-29)
        if (row_values.size() >= 30) {
            data.acc_body_x.push_back(row_values[24]);
            data.acc_body_y.push_back(row_values[25]);
            data.acc_body_z.push_back(row_values[26]);
            data.gyro_body_x.push_back(row_values[27]);
            data.gyro_body_y.push_back(row_values[28]);
            data.gyro_body_z.push_back(row_values[29]);
        }
        
        // Contact planning and phase (30-34)
        if (row_values.size() >= 35) {
            data.lfoot_contact_plan.push_back(row_values[30]);
            data.lfoot_contact.push_back(row_values[31]);
            data.rfoot_contact_plan.push_back(row_values[32]);
            data.rfoot_contact.push_back(row_values[33]);
            data.phase_variable.push_back(row_values[34]);
        }
        
        // Command velocities and stability (35-40)
        if (row_values.size() >= 41) {
            data.cmd_vel_x.push_back(row_values[35]);
            data.cmd_vel_y.push_back(row_values[36]);
            data.cmd_vel_z.push_back(row_values[37]);
            data.cmd_effective.push_back(row_values[38]);
            data.cmd_turning.push_back(row_values[39]);
            data.stability.push_back(row_values[40]);
        }
        
        // Right foot ground reaction forces (41-46)
        if (row_values.size() >= 47) {
            data.rfoot_grf_x.push_back(row_values[41]);
            data.rfoot_grf_y.push_back(row_values[42]);
            data.rfoot_grf_z.push_back(row_values[43]);
            data.rfoot_grt_x.push_back(row_values[44]);
            data.rfoot_grt_y.push_back(row_values[45]);
            data.rfoot_grt_z.push_back(row_values[46]);
        }
        
        // Left foot ground reaction forces (47-52)
        if (row_values.size() >= 53) {
            data.lfoot_grf_x.push_back(row_values[47]);
            data.lfoot_grf_y.push_back(row_values[48]);
            data.lfoot_grf_z.push_back(row_values[49]);
            data.lfoot_grt_x.push_back(row_values[50]);
            data.lfoot_grt_y.push_back(row_values[51]);
            data.lfoot_grt_z.push_back(row_values[52]);
        }
        
        // Right foot pose (53-59)
        if (row_values.size() >= 60) {
            data.rfoot_pos_x.push_back(row_values[53]);
            data.rfoot_pos_y.push_back(row_values[54]);
            data.rfoot_pos_z.push_back(row_values[55]);
            data.rfoot_quat_w.push_back(row_values[56]);
            data.rfoot_quat_i.push_back(row_values[57]);
            data.rfoot_quat_j.push_back(row_values[58]);
            data.rfoot_quat_k.push_back(row_values[59]);
        }
        
        // Left foot pose (60-66)
        if (row_values.size() >= 67) {
            data.lfoot_pos_x.push_back(row_values[60]);
            data.lfoot_pos_y.push_back(row_values[61]);
            data.lfoot_pos_z.push_back(row_values[62]);
            data.lfoot_quat_w.push_back(row_values[63]);
            data.lfoot_quat_i.push_back(row_values[64]);
            data.lfoot_quat_j.push_back(row_values[65]);
            data.lfoot_quat_k.push_back(row_values[66]);
        }
        
        // Joint positions (67-109)
        if (row_values.size() >= 110) {
            data.joint_pos_left_hip_pitch.push_back(row_values[67]);
            data.joint_pos_right_hip_pitch.push_back(row_values[68]);
            data.joint_pos_waist_yaw.push_back(row_values[69]);
            data.joint_pos_left_hip_roll.push_back(row_values[70]);
            data.joint_pos_right_hip_roll.push_back(row_values[71]);
            data.joint_pos_waist_roll.push_back(row_values[72]);
            data.joint_pos_left_hip_yaw.push_back(row_values[73]);
            data.joint_pos_right_hip_yaw.push_back(row_values[74]);
            data.joint_pos_waist_pitch.push_back(row_values[75]);
            data.joint_pos_left_knee.push_back(row_values[76]);
            data.joint_pos_right_knee.push_back(row_values[77]);
            data.joint_pos_left_shoulder_pitch.push_back(row_values[78]);
            data.joint_pos_right_shoulder_pitch.push_back(row_values[79]);
            data.joint_pos_left_ankle_pitch.push_back(row_values[80]);
            data.joint_pos_right_ankle_pitch.push_back(row_values[81]);
            data.joint_pos_left_shoulder_roll.push_back(row_values[82]);
            data.joint_pos_right_shoulder_roll.push_back(row_values[83]);
            data.joint_pos_left_ankle_roll.push_back(row_values[84]);
            data.joint_pos_right_ankle_roll.push_back(row_values[85]);
            data.joint_pos_left_shoulder_yaw.push_back(row_values[86]);
            data.joint_pos_right_shoulder_yaw.push_back(row_values[87]);
            data.joint_pos_left_elbow.push_back(row_values[88]);
            data.joint_pos_right_elbow.push_back(row_values[89]);
            data.joint_pos_left_wrist_roll.push_back(row_values[90]);
            data.joint_pos_right_wrist_roll.push_back(row_values[91]);
            data.joint_pos_left_wrist_pitch.push_back(row_values[92]);
            data.joint_pos_right_wrist_pitch.push_back(row_values[93]);
            data.joint_pos_left_wrist_yaw.push_back(row_values[94]);
            data.joint_pos_right_wrist_yaw.push_back(row_values[95]);
            data.joint_pos_left_hand_index_0.push_back(row_values[96]);
            data.joint_pos_left_hand_middle_0.push_back(row_values[97]);
            data.joint_pos_left_hand_thumb_0.push_back(row_values[98]);
            data.joint_pos_right_hand_index_0.push_back(row_values[99]);
            data.joint_pos_right_hand_middle_0.push_back(row_values[100]);
            data.joint_pos_right_hand_thumb_0.push_back(row_values[101]);
            data.joint_pos_left_hand_index_1.push_back(row_values[102]);
            data.joint_pos_left_hand_middle_1.push_back(row_values[103]);
            data.joint_pos_left_hand_thumb_1.push_back(row_values[104]);
            data.joint_pos_right_hand_index_1.push_back(row_values[105]);
            data.joint_pos_right_hand_middle_1.push_back(row_values[106]);
            data.joint_pos_right_hand_thumb_1.push_back(row_values[107]);
            data.joint_pos_left_hand_thumb_2.push_back(row_values[108]);
            data.joint_pos_right_hand_thumb_2.push_back(row_values[109]);
        }
        
        // Joint velocities (110-152)
        if (row_values.size() >= 153) {
            data.joint_vel_left_hip_pitch.push_back(row_values[110]);
            data.joint_vel_right_hip_pitch.push_back(row_values[111]);
            data.joint_vel_waist_yaw.push_back(row_values[112]);
            data.joint_vel_left_hip_roll.push_back(row_values[113]);
            data.joint_vel_right_hip_roll.push_back(row_values[114]);
            data.joint_vel_waist_roll.push_back(row_values[115]);
            data.joint_vel_left_hip_yaw.push_back(row_values[116]);
            data.joint_vel_right_hip_yaw.push_back(row_values[117]);
            data.joint_vel_waist_pitch.push_back(row_values[118]);
            data.joint_vel_left_knee.push_back(row_values[119]);
            data.joint_vel_right_knee.push_back(row_values[120]);
            data.joint_vel_left_shoulder_pitch.push_back(row_values[121]);
            data.joint_vel_right_shoulder_pitch.push_back(row_values[122]);
            data.joint_vel_left_ankle_pitch.push_back(row_values[123]);
            data.joint_vel_right_ankle_pitch.push_back(row_values[124]);
            data.joint_vel_left_shoulder_roll.push_back(row_values[125]);
            data.joint_vel_right_shoulder_roll.push_back(row_values[126]);
            data.joint_vel_left_ankle_roll.push_back(row_values[127]);
            data.joint_vel_right_ankle_roll.push_back(row_values[128]);
            data.joint_vel_left_shoulder_yaw.push_back(row_values[129]);
            data.joint_vel_right_shoulder_yaw.push_back(row_values[130]);
            data.joint_vel_left_elbow.push_back(row_values[131]);
            data.joint_vel_right_elbow.push_back(row_values[132]);
            data.joint_vel_left_wrist_roll.push_back(row_values[133]);
            data.joint_vel_right_wrist_roll.push_back(row_values[134]);
            data.joint_vel_left_wrist_pitch.push_back(row_values[135]);
            data.joint_vel_right_wrist_pitch.push_back(row_values[136]);
            data.joint_vel_left_wrist_yaw.push_back(row_values[137]);
            data.joint_vel_right_wrist_yaw.push_back(row_values[138]);
            data.joint_vel_left_hand_index_0.push_back(row_values[139]);
            data.joint_vel_left_hand_middle_0.push_back(row_values[140]);
            data.joint_vel_left_hand_thumb_0.push_back(row_values[141]);
            data.joint_vel_right_hand_index_0.push_back(row_values[142]);
            data.joint_vel_right_hand_middle_0.push_back(row_values[143]);
            data.joint_vel_right_hand_thumb_0.push_back(row_values[144]);
            data.joint_vel_left_hand_index_1.push_back(row_values[145]);
            data.joint_vel_left_hand_middle_1.push_back(row_values[146]);
            data.joint_vel_left_hand_thumb_1.push_back(row_values[147]);
            data.joint_vel_right_hand_index_1.push_back(row_values[148]);
            data.joint_vel_right_hand_middle_1.push_back(row_values[149]);
            data.joint_vel_right_hand_thumb_1.push_back(row_values[150]);
            data.joint_vel_left_hand_thumb_2.push_back(row_values[151]);
            data.joint_vel_right_hand_thumb_2.push_back(row_values[152]);
        }
        
        // Joint torques (153-195)
        if (row_values.size() >= 196) {
            data.joint_torque_left_hip_pitch.push_back(row_values[153]);
            data.joint_torque_right_hip_pitch.push_back(row_values[154]);
            data.joint_torque_waist_yaw.push_back(row_values[155]);
            data.joint_torque_left_hip_roll.push_back(row_values[156]);
            data.joint_torque_right_hip_roll.push_back(row_values[157]);
            data.joint_torque_waist_roll.push_back(row_values[158]);
            data.joint_torque_left_hip_yaw.push_back(row_values[159]);
            data.joint_torque_right_hip_yaw.push_back(row_values[160]);
            data.joint_torque_waist_pitch.push_back(row_values[161]);
            data.joint_torque_left_knee.push_back(row_values[162]);
            data.joint_torque_right_knee.push_back(row_values[163]);
            data.joint_torque_left_shoulder_pitch.push_back(row_values[164]);
            data.joint_torque_right_shoulder_pitch.push_back(row_values[165]);
            data.joint_torque_left_ankle_pitch.push_back(row_values[166]);
            data.joint_torque_right_ankle_pitch.push_back(row_values[167]);
            data.joint_torque_left_shoulder_roll.push_back(row_values[168]);
            data.joint_torque_right_shoulder_roll.push_back(row_values[169]);
            data.joint_torque_left_ankle_roll.push_back(row_values[170]);
            data.joint_torque_right_ankle_roll.push_back(row_values[171]);
            data.joint_torque_left_shoulder_yaw.push_back(row_values[172]);
            data.joint_torque_right_shoulder_yaw.push_back(row_values[173]);
            data.joint_torque_left_elbow.push_back(row_values[174]);
            data.joint_torque_right_elbow.push_back(row_values[175]);
            data.joint_torque_left_wrist_roll.push_back(row_values[176]);
            data.joint_torque_right_wrist_roll.push_back(row_values[177]);
            data.joint_torque_left_wrist_pitch.push_back(row_values[178]);
            data.joint_torque_right_wrist_pitch.push_back(row_values[179]);
            data.joint_torque_left_wrist_yaw.push_back(row_values[180]);
            data.joint_torque_right_wrist_yaw.push_back(row_values[181]);
            data.joint_torque_left_hand_index_0.push_back(row_values[182]);
            data.joint_torque_left_hand_middle_0.push_back(row_values[183]);
            data.joint_torque_left_hand_thumb_0.push_back(row_values[184]);
            data.joint_torque_right_hand_index_0.push_back(row_values[185]);
            data.joint_torque_right_hand_middle_0.push_back(row_values[186]);
            data.joint_torque_right_hand_thumb_0.push_back(row_values[187]);
            data.joint_torque_left_hand_index_1.push_back(row_values[188]);
            data.joint_torque_left_hand_middle_1.push_back(row_values[189]);
            data.joint_torque_left_hand_thumb_1.push_back(row_values[190]);
            data.joint_torque_right_hand_index_1.push_back(row_values[191]);
            data.joint_torque_right_hand_middle_1.push_back(row_values[192]);
            data.joint_torque_right_hand_thumb_1.push_back(row_values[193]);
            data.joint_torque_left_hand_thumb_2.push_back(row_values[194]);
            data.joint_torque_right_hand_thumb_2.push_back(row_values[195]);
        }
        
        // Joint actions (196-224) - 29 joints for actions  
        if (row_values.size() >= 225) {
            data.joint_action_left_hip_pitch.push_back(row_values[196]);
            data.joint_action_left_hip_roll.push_back(row_values[197]);
            data.joint_action_left_hip_yaw.push_back(row_values[198]);
            data.joint_action_left_knee.push_back(row_values[199]);
            data.joint_action_left_ankle_pitch.push_back(row_values[200]);
            data.joint_action_left_ankle_roll.push_back(row_values[201]);
            data.joint_action_right_hip_pitch.push_back(row_values[202]);
            data.joint_action_right_hip_roll.push_back(row_values[203]);
            data.joint_action_right_hip_yaw.push_back(row_values[204]);
            data.joint_action_right_knee.push_back(row_values[205]);
            data.joint_action_right_ankle_pitch.push_back(row_values[206]);
            data.joint_action_right_ankle_roll.push_back(row_values[207]);
            data.joint_action_waist_yaw.push_back(row_values[208]);
            data.joint_action_waist_roll.push_back(row_values[209]);
            data.joint_action_waist_pitch.push_back(row_values[210]);
            data.joint_action_left_shoulder_pitch.push_back(row_values[211]);
            data.joint_action_left_shoulder_roll.push_back(row_values[212]);
            data.joint_action_left_shoulder_yaw.push_back(row_values[213]);
            data.joint_action_left_elbow.push_back(row_values[214]);
            data.joint_action_left_wrist_roll.push_back(row_values[215]);
            data.joint_action_left_wrist_pitch.push_back(row_values[216]);
            data.joint_action_left_wrist_yaw.push_back(row_values[217]);
            data.joint_action_right_shoulder_pitch.push_back(row_values[218]);
            data.joint_action_right_shoulder_roll.push_back(row_values[219]);
            data.joint_action_right_shoulder_yaw.push_back(row_values[220]);
            data.joint_action_right_elbow.push_back(row_values[221]);
            data.joint_action_right_wrist_roll.push_back(row_values[222]);
            data.joint_action_right_wrist_pitch.push_back(row_values[223]);
            data.joint_action_right_wrist_yaw.push_back(row_values[224]);
        }
        
        row_count++;
    }
    
    file.close();
    data.num_rows = row_count;
    
    std::cout << "Parsed " << row_count << " rows from G1 CSV file" << std::endl;
    std::cout << "Total columns parsed: 225 (count, timestamp, and 223 data columns)" << std::endl;
    
    return data;
}
