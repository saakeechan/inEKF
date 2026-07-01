#include "inekf_save_utils.hpp"
#include "../estimator/InEKF_estimator.hpp"
#include "BasicFunctions_Estimator.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <chrono>
#include <sstream>

using std::cout;
using std::endl;

namespace InEKF_SaveUtils {

void save_onestep_impl(InEKFilter& filter, int save_index) {
    
    if(save_index >= filter.SAVEMAXCNT) {
        save_index = filter.SAVEMAXCNT - 1;
        cout << "over Max_SAVE_cnt!!" << endl;
        return;
    }
    
    // Save estimated dv values (always, regardless of csv_mode)
    for(int i=0; i<filter.robot.num_legs; i++) {
        for(int k=0; k<3; k++) {
            filter.SAVE_MATRIX[filter.idx_ESTIMATED_dv + 3 * i + k][save_index] = 
                filter.d_v[filter.frame_count](3*i + k);
        }
    }
    for (int i=0; i<filter.robot.num_legs; i++) {
        filter.SAVE_MATRIX[filter.idx_ESTIMATED_dv_norm + i][save_index] = filter.dv_norm[filter.frame_count](i);
    }
    
    if(filter.csv_mode) {
        //Sending True States----------------------------------------------------------------------------
        int j = filter.time_count + filter.start_index + 1;

        // Build rotation matrix from CSV quaternion data
        Eigen::Vector4d quat_true;
        quat_true << filter.CSVData.quat_w[j], filter.CSVData.quat_i[j], 
                     filter.CSVData.quat_j[j], filter.CSVData.quat_k[j];
        Eigen::Matrix3d temp_R = BasicFunctions_Estimator::Quaternion_to_Rotation_Matrix(quat_true);

        Eigen::Vector3d temp_eul = BasicFunctions_Estimator::Rotation_to_EulerZYX(temp_R);

        for(int i=0; i<3; i++) {
            for(int k=0; k<3; k++) {
                filter.SAVE_MATRIX[filter.idx_TRUE_Rotation+i*3+k][save_index] = temp_R(i,k);
            }
        }

        // Calculate ground truth foot velocities using finite differences from CSV foot positions
        Eigen::Vector<double,3> dv_true;

        for(int i=0; i<filter.robot.num_legs; i++) {
            // Get foot positions from CSV (i=0 is left foot, i=1 is right foot, etc.)
            Eigen::Vector3d foot_pos_current, foot_pos_next;
            
            if (i == 0) {  // Left foot
                foot_pos_current << filter.CSVData.lfoot_pos_x[j], 
                                     filter.CSVData.lfoot_pos_y[j], 
                                     filter.CSVData.lfoot_pos_z[j];
                
                if (j < filter.CSVData.lfoot_pos_x.size() - 1) {
                    // Forward difference: (pos[j+1] - pos[j]) / dt
                    foot_pos_next << filter.CSVData.lfoot_pos_x[j+1], 
                                     filter.CSVData.lfoot_pos_y[j+1], 
                                     filter.CSVData.lfoot_pos_z[j+1];
                    dv_true = (foot_pos_next - foot_pos_current) / filter.dt;
                } else {
                    // For last timestep, use the velocity from previous timestep
                    for(int k=0; k<3; k++) {
                        dv_true(k) = filter.SAVE_MATRIX[filter.idx_TRUE_dv + 3 * i + k][save_index - 1];
                    }
                }
            } else if (i == 1) {  // Right foot
                foot_pos_current << filter.CSVData.rfoot_pos_x[j], 
                                     filter.CSVData.rfoot_pos_y[j], 
                                     filter.CSVData.rfoot_pos_z[j];
                
                if (j < filter.CSVData.rfoot_pos_x.size() - 1) {
                    // Forward difference: (pos[j+1] - pos[j]) / dt
                    foot_pos_next << filter.CSVData.rfoot_pos_x[j+1], 
                                     filter.CSVData.rfoot_pos_y[j+1], 
                                     filter.CSVData.rfoot_pos_z[j+1];
                    dv_true = (foot_pos_next - foot_pos_current) / filter.dt;
                } else {
                    // For last timestep, use the velocity from previous timestep
                    for(int k=0; k<3; k++) {
                        dv_true(k) = filter.SAVE_MATRIX[filter.idx_TRUE_dv + 3 * i + k][save_index - 1];
                    }
                }
            }

            for(int k=0; k<3; k++) {
                filter.SAVE_MATRIX[filter.idx_TRUE_dv + 3 * i + k][save_index] = dv_true(k);
            }
            
            // Save TRUE dv_norm
            filter.SAVE_MATRIX[filter.idx_TRUE_dv_norm + i][save_index] = dv_true.norm();
        }

        // Save position, velocity, angular velocity (omega), biases, and RPY
        for(int i=0; i<3; i++) {
            filter.SAVE_MATRIX[filter.idx_TRUE_Position+i][save_index] = 
                filter.CSVData.pos_x[j] * (i == 0) + 
                filter.CSVData.pos_y[j] * (i == 1) + 
                filter.CSVData.pos_z[j] * (i == 2);
            filter.SAVE_MATRIX[filter.idx_TRUE_Velocity+i][save_index] = 
                filter.CSVData.vel_x[j] * (i == 0) + 
                filter.CSVData.vel_y[j] * (i == 1) + 
                filter.CSVData.vel_z[j] * (i == 2);
            filter.SAVE_MATRIX[filter.idx_TRUE_Omega+i][save_index] = 
                filter.CSVData.omega_x[j] * (i == 0) + 
                filter.CSVData.omega_y[j] * (i == 1) + 
                filter.CSVData.omega_z[j] * (i == 2);
            filter.SAVE_MATRIX[filter.idx_TRUE_Bias_Gyro+i][save_index] = 0;
            filter.SAVE_MATRIX[filter.idx_TRUE_Bias_Acc+i][save_index] = 0;
            filter.SAVE_MATRIX[filter.idx_TRUE_rpy+i][save_index] = temp_eul(i);
        }

        // Save contact states
        std::vector<bool> temp_contact = filter.CSVData.contact_state[j];
        
        for(int i=0; i<filter.robot.num_legs; i++) {
            filter.SAVE_MATRIX[filter.idx_TRUE_Contact+i][save_index] = temp_contact[i];
            filter.SAVE_MATRIX[filter.idx_TRUE_Slip+i][save_index] = filter.SLIP_t[filter.frame_count][i];
            filter.SAVE_MATRIX[filter.idx_TRUE_Hard_Contact+i][save_index] = filter.HARD_CONTACT_t[filter.frame_count][i];
        }
    }

    // Save estimated states (always done, regardless of csv_mode)
    for(int i=0; i<filter.robot.num_legs; i++) {
        filter.SAVE_MATRIX[filter.idx_ESTIMATED_Contact+i][save_index] = filter.CONTACT_t[filter.frame_count][i];
        filter.SAVE_MATRIX[filter.idx_ESTIMATED_Slip+i][save_index] = filter.SLIP_t[filter.frame_count][i];
        filter.SAVE_MATRIX[filter.idx_ESTIMATED_Hard_Contact+i][save_index] = filter.HARD_CONTACT_t[filter.frame_count][i];
    }

    Eigen::Vector3d temp_eul = BasicFunctions_Estimator::Rotation_to_EulerZYX(filter.robot_state.Rotation);
    for(int i=0; i<3; i++) {
        filter.SAVE_MATRIX[filter.idx_ESTIMATED_Position+i][save_index] = filter.robot_state.Position(i);
        filter.SAVE_MATRIX[filter.idx_ESTIMATED_Velocity+i][save_index] = filter.robot_state.Velocity(i);
        filter.SAVE_MATRIX[filter.idx_ESTIMATED_Omega+i][save_index] = filter.robot_state.Angular_Velocity(i);
        filter.SAVE_MATRIX[filter.idx_ESTIMATED_Bias_Gyro+i][save_index] = filter.robot_state.Bias_Gyro(i);
        filter.SAVE_MATRIX[filter.idx_ESTIMATED_Bias_Acc+i][save_index] = filter.robot_state.Bias_Acc(i);
        filter.SAVE_MATRIX[filter.idx_ESTIMATED_rpy+i][save_index] = temp_eul(i);
    }

    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            filter.SAVE_MATRIX[filter.idx_ESTIMATED_Rotation+i*3+j][save_index] = 
                filter.robot_state.Rotation(i,j);
        }
    }
}

void print_contact_accuracy_impl(const InEKFilter& filter) {
    if (filter.total_comparisons <= 0) {
        return;
    }

    cout << "\n=== Contact Detection Accuracy ==="<< endl;
    cout << "Total comparisons: " << filter.total_comparisons << endl;
    for (int k = 0; k < filter.robot.num_legs; k++) {
        int total_leg = filter.contact_match_count[k] + filter.contact_mismatch_count[k];
        double accuracy = (total_leg > 0) ? (100.0 * filter.contact_match_count[k] / total_leg) : 0.0;
        cout << "Leg " << (k+1) << ": Matches=" << filter.contact_match_count[k] 
             << ", Mismatches=" << filter.contact_mismatch_count[k]
             << ", Accuracy=" << std::fixed << std::setprecision(2) << accuracy << "%" << endl;
    }
}

void write_saved_data_impl(InEKFilter& filter) {
    
    // Create output directory if it doesn't exist
    try {
        std::filesystem::create_directories(filter.saved_results_folder);
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Error: Failed to create directory: " << filter.saved_results_folder 
                  << " - " << e.what() << std::endl;
        return;
    }

    // Get current date and time for filename
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm* tm_now = std::localtime(&time_t_now);
    
    // Format: YYYY-MM-DD_HH-MM-SS
    std::stringstream timestamp;
    timestamp << std::put_time(tm_now, "%Y-%m-%d_%H-%M-%S");
    std::string timestamp_str = timestamp.str();

    // Create results_meta folder
    std::string meta_folder = filter.saved_results_folder + "/results_meta";
    try {
        std::filesystem::create_directories(meta_folder);
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Error: Failed to create directory: " << meta_folder 
                  << " - " << e.what() << std::endl;
    }

    // Build filename with timestamp
    std::string filename = filter.saved_results_folder + "/" + timestamp_str + ".csv";
    
    // Build metadata filename
    std::string meta_filename = meta_folder + "/" + timestamp_str + "_info.txt";
    
    // Write metadata to text file
    std::ofstream metafile(meta_filename);
    if (metafile.is_open()) {
        metafile << "Estimator Info: " << filter.estimator_info << "\n";
        metafile << "Covariance Info: " << filter.cov_info << "\n";
        metafile << "Initial Condition Info: " << filter.initial_info << "\n";
        metafile << "File Info: " << filter.file_info << "\n";
        metafile << "Time Count: " << filter.time_count << "\n";
        metafile << "Timestamp: " << timestamp_str << "\n";
        metafile << "dt: " << filter.dt << "\n";
        metafile << "Robot Name: " << filter.robot.robot_name << "\n";
        metafile << "Number of Legs: " << filter.robot.num_legs << "\n";
        metafile << "Slip Rejection Mode: " << (filter.slip_rejection_mode ? "true" : "false") << "\n";
        metafile << "Slip Threshold: " << filter.slip_threshold << "\n";
        metafile << "Variable Contact Cov Mode: " << (filter.variable_contact_cov_mode ? "true" : "false") << "\n";
        metafile << "Cov Amplifier: " << filter.cov_amplifier << "\n";
        metafile << "Contact NN Enabled: " << (filter.contact_nn_enabled ? "true" : "false") << "\n";
        if (filter.contact_nn_enabled) {
            metafile << "Contact Model Path: " << filter.contact_model_path << "\n";
        }
        metafile.close();
        cout << "Metadata saved to: " << meta_filename << endl;
    } else {
        std::cerr << "Error: Failed to open metadata file: " << meta_filename << std::endl;
    }
    
    // Use C++ file stream with RAII and error checking
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "Error: Failed to open file: " << filename << std::endl;
        return;
    }

    // Write CSV headers
    std::vector<std::string> headers;
    
    // Time
    headers.push_back("time");
    
    // ESTIMATED values
    headers.push_back("EST_Px"); headers.push_back("EST_Py"); headers.push_back("EST_Pz");
    headers.push_back("EST_Qw"); headers.push_back("EST_Qx"); headers.push_back("EST_Qy"); headers.push_back("EST_Qz");
    headers.push_back("EST_roll"); headers.push_back("EST_pitch"); headers.push_back("EST_yaw");
    headers.push_back("EST_Vx"); headers.push_back("EST_Vy"); headers.push_back("EST_Vz");
    headers.push_back("EST_Omega_x"); headers.push_back("EST_Omega_y"); headers.push_back("EST_Omega_z");
    headers.push_back("EST_Bg_x"); headers.push_back("EST_Bg_y"); headers.push_back("EST_Bg_z");
    headers.push_back("EST_Ba_x"); headers.push_back("EST_Ba_y"); headers.push_back("EST_Ba_z");
    for(int k=0; k<filter.robot.num_legs; k++) headers.push_back("EST_Contact_leg" + std::to_string(k+1));
    for(int k=0; k<filter.robot.num_legs; k++) {
        headers.push_back("EST_dv_leg" + std::to_string(k+1) + "_x");
        headers.push_back("EST_dv_leg" + std::to_string(k+1) + "_y");
        headers.push_back("EST_dv_leg" + std::to_string(k+1) + "_z");
    }
    for(int k=0; k<filter.robot.num_legs; k++) headers.push_back("EST_dv_norm_leg" + std::to_string(k+1));
    
    // Ground truth values (only if csv mode)
    if (filter.csv_mode) {
        headers.push_back("GT_Px"); headers.push_back("GT_Py"); headers.push_back("GT_Pz");
        headers.push_back("GT_Qw"); headers.push_back("GT_Qx"); headers.push_back("GT_Qy"); headers.push_back("GT_Qz");
        headers.push_back("GT_roll"); headers.push_back("GT_pitch"); headers.push_back("GT_yaw");
        headers.push_back("GT_Vx"); headers.push_back("GT_Vy"); headers.push_back("GT_Vz");
        headers.push_back("GT_Omega_x"); headers.push_back("GT_Omega_y"); headers.push_back("GT_Omega_z");
        for(int k=0; k<filter.robot.num_legs; k++) headers.push_back("GT_Contact_leg" + std::to_string(k+1));
        for(int k=0; k<filter.robot.num_legs; k++) {
            headers.push_back("GT_dv_leg" + std::to_string(k+1) + "_x");
            headers.push_back("GT_dv_leg" + std::to_string(k+1) + "_y");
            headers.push_back("GT_dv_leg" + std::to_string(k+1) + "_z");
        }
        for(int k=0; k<filter.robot.num_legs; k++) headers.push_back("GT_dv_norm_leg" + std::to_string(k+1));
    }
    
    // Write header row
    for(size_t h = 0; h < headers.size(); h++) {
        outfile << headers[h];
        if(h < headers.size() - 1) outfile << ",";
    }
    outfile << "\n";

    // Write data rows
    for(int i = 0; i < filter.time_count; i++) {
        // Time
        outfile << (i * filter.dt);
        
        // ESTIMATED - Position
        outfile << "," << filter.SAVE_MATRIX[filter.idx_ESTIMATED_Position][i];
        outfile << "," << filter.SAVE_MATRIX[filter.idx_ESTIMATED_Position+1][i];
        outfile << "," << filter.SAVE_MATRIX[filter.idx_ESTIMATED_Position+2][i];
        
        // ESTIMATED - Rotation as quaternion (convert from rotation matrix)
        Eigen::Matrix3d R_est;
        for(int r=0; r<3; r++) for(int c=0; c<3; c++) 
            R_est(r,c) = filter.SAVE_MATRIX[filter.idx_ESTIMATED_Rotation + r*3 + c][i];
        Eigen::Vector4d quat_est = BasicFunctions_Estimator::Rotation_Matrix_to_Quaternion(R_est);
        outfile << "," << quat_est(0) << "," << quat_est(1) << "," << quat_est(2) << "," << quat_est(3);
        
        // ESTIMATED - Roll, Pitch, Yaw
        outfile << "," << filter.SAVE_MATRIX[filter.idx_ESTIMATED_rpy][i];
        outfile << "," << filter.SAVE_MATRIX[filter.idx_ESTIMATED_rpy+1][i];
        outfile << "," << filter.SAVE_MATRIX[filter.idx_ESTIMATED_rpy+2][i];
        
        // ESTIMATED - Velocity
        outfile << "," << filter.SAVE_MATRIX[filter.idx_ESTIMATED_Velocity][i];
        outfile << "," << filter.SAVE_MATRIX[filter.idx_ESTIMATED_Velocity+1][i];
        outfile << "," << filter.SAVE_MATRIX[filter.idx_ESTIMATED_Velocity+2][i];
        
        // ESTIMATED - Omega (Angular Velocity)
        outfile << "," << filter.SAVE_MATRIX[filter.idx_ESTIMATED_Omega][i];
        outfile << "," << filter.SAVE_MATRIX[filter.idx_ESTIMATED_Omega+1][i];
        outfile << "," << filter.SAVE_MATRIX[filter.idx_ESTIMATED_Omega+2][i];
        
        // ESTIMATED - Biases
        outfile << "," << filter.SAVE_MATRIX[filter.idx_ESTIMATED_Bias_Gyro][i];
        outfile << "," << filter.SAVE_MATRIX[filter.idx_ESTIMATED_Bias_Gyro+1][i];
        outfile << "," << filter.SAVE_MATRIX[filter.idx_ESTIMATED_Bias_Gyro+2][i];
        outfile << "," << filter.SAVE_MATRIX[filter.idx_ESTIMATED_Bias_Acc][i];
        outfile << "," << filter.SAVE_MATRIX[filter.idx_ESTIMATED_Bias_Acc+1][i];
        outfile << "," << filter.SAVE_MATRIX[filter.idx_ESTIMATED_Bias_Acc+2][i];
        
        // ESTIMATED - Contact
        for(int k=0; k<filter.robot.num_legs; k++) 
            outfile << "," << filter.SAVE_MATRIX[filter.idx_ESTIMATED_Contact+k][i];
        
        // ESTIMATED - dv per leg
        for(int k=0; k<filter.robot.num_legs; k++) {
            outfile << "," << filter.SAVE_MATRIX[filter.idx_ESTIMATED_dv + 3*k][i];
            outfile << "," << filter.SAVE_MATRIX[filter.idx_ESTIMATED_dv + 3*k + 1][i];
            outfile << "," << filter.SAVE_MATRIX[filter.idx_ESTIMATED_dv + 3*k + 2][i];
        }
        
        // ESTIMATED - dv_norm per leg (from SAVE_MATRIX)
        for(int k=0; k<filter.robot.num_legs; k++) {
            outfile << "," << filter.SAVE_MATRIX[filter.idx_ESTIMATED_dv_norm + k][i];
        }
        
        // Ground truth data (only if csv mode)
        if (filter.csv_mode) {
            // GT - Position
            outfile << "," << filter.SAVE_MATRIX[filter.idx_TRUE_Position][i];
            outfile << "," << filter.SAVE_MATRIX[filter.idx_TRUE_Position+1][i];
            outfile << "," << filter.SAVE_MATRIX[filter.idx_TRUE_Position+2][i];
            
            // GT - Rotation as quaternion
            Eigen::Matrix3d R_gt;
            for(int r=0; r<3; r++) for(int c=0; c<3; c++) 
                R_gt(r,c) = filter.SAVE_MATRIX[filter.idx_TRUE_Rotation + r*3 + c][i];
            Eigen::Vector4d quat_gt = BasicFunctions_Estimator::Rotation_Matrix_to_Quaternion(R_gt);
            outfile << "," << quat_gt(0) << "," << quat_gt(1) << "," << quat_gt(2) << "," << quat_gt(3);
            
            // GT - Roll, Pitch, Yaw
            outfile << "," << filter.SAVE_MATRIX[filter.idx_TRUE_rpy][i];
            outfile << "," << filter.SAVE_MATRIX[filter.idx_TRUE_rpy+1][i];
            outfile << "," << filter.SAVE_MATRIX[filter.idx_TRUE_rpy+2][i];
            
            // GT - Velocity
            outfile << "," << filter.SAVE_MATRIX[filter.idx_TRUE_Velocity][i];
            outfile << "," << filter.SAVE_MATRIX[filter.idx_TRUE_Velocity+1][i];
            outfile << "," << filter.SAVE_MATRIX[filter.idx_TRUE_Velocity+2][i];
            
            // GT - Omega (Angular Velocity)
            outfile << "," << filter.SAVE_MATRIX[filter.idx_TRUE_Omega][i];
            outfile << "," << filter.SAVE_MATRIX[filter.idx_TRUE_Omega+1][i];
            outfile << "," << filter.SAVE_MATRIX[filter.idx_TRUE_Omega+2][i];
            
            // GT - Contact
            for(int k=0; k<filter.robot.num_legs; k++) 
                outfile << "," << filter.SAVE_MATRIX[filter.idx_TRUE_Contact+k][i];
            
            // GT - dv per leg (from SAVE_MATRIX)
            for(int k=0; k<filter.robot.num_legs; k++) {
                outfile << "," << filter.SAVE_MATRIX[filter.idx_TRUE_dv + 3*k][i];
                outfile << "," << filter.SAVE_MATRIX[filter.idx_TRUE_dv + 3*k + 1][i];
                outfile << "," << filter.SAVE_MATRIX[filter.idx_TRUE_dv + 3*k + 2][i];
            }
            
            // GT - dv_norm per leg (from SAVE_MATRIX)
            for(int k=0; k<filter.robot.num_legs; k++) {
                outfile << "," << filter.SAVE_MATRIX[filter.idx_TRUE_dv_norm + k][i];
            }
        }
        
        outfile << "\n";
    }

    // File automatically closed by destructor
    cout << filename << endl;
    cout << "Saving to a file done" << endl;
    
}

} // namespace InEKF_SaveUtils
