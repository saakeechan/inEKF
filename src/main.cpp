
#include <iostream>
#include <string>
#include <algorithm>

#include <cmath> // for std::pow
#include <chrono> // for timing

#include <eigen3/Eigen/Dense>
#include <vector>
#include "estimator/InEKF_estimator.hpp"
#include "utility/inekf_save_utils.hpp"
#include <iomanip>
#include "utility/RobotModel.hpp"


using std::cout;
using std::endl;

/// @brief 
/// @return 
int main() {


    // If this is your first time running a new robot, uncomment the line below
    //  to extract parameters and use that to fill in the yaml files
    
    // InEKFilter::getParameters("../src/utility/G1/g1_29dof.xml", "g1_wFootFrames_XML");
    // return 0;


    // InEKFilter::getParameters("../digit_mujoco-main/src/digit_mujoco/models/flat/digit-v3-flat.xml", "Digit");
    // return 0;

    InEKFilter estimator_IEKF("../config/initialization_config.yaml", "../config/G1_config.yaml");

    // // Create random Eigen vectors for IMU, motor data (2×num_joints matrix), and torque values
    // Eigen::VectorXd imu_values(6);
    // Eigen::MatrixXd motor_data(2, 29);  // Row 0: positions, Row 1: velocities
    // Eigen::VectorXd torque_values(29);

    // // Initialize random values
    // for (int i = 0; i < 6; i++) {
    //     imu_values(i) = static_cast<double>(rand()) / RAND_MAX;
    // }
    // for (int i = 0; i < 29; i++) {
    //     motor_data(0, i) = static_cast<double>(rand()) / RAND_MAX;  // Position
    //     motor_data(1, i) = static_cast<double>(rand()) / RAND_MAX;  // Velocity
    //     torque_values(i) = static_cast<double>(rand()) / RAND_MAX;
    // }

    // Eigen::VectorXd motor_torque_cmd(29);
    // Eigen::VectorXd cmd_vel(3);

    // // Initialize random motor torque commands
    // for (int i = 0; i < 29; i++) {
    //     motor_torque_cmd(i) = static_cast<double>(rand()) / RAND_MAX;
    // }
    
    // // Initialize random command velocities
    // for (int i = 0; i < 3; i++) {
    //     cmd_vel(i) = static_cast<double>(rand()) / RAND_MAX;
    // }

    std::vector<double> step_times;

        for (int time = 1; time < (estimator_IEKF.time_length - estimator_IEKF.start_index); time++) {
            auto start = std::chrono::high_resolution_clock::now();

            estimator_IEKF.Onestep();  // Contact detection from torques
            std::cout << "Loop " << time;
            if (estimator_IEKF.csv_mode) {
                const int csv_index = estimator_IEKF.start_index + estimator_IEKF.time_count;
                if (csv_index >= 0 && csv_index < static_cast<int>(estimator_IEKF.CSVData.timestamp.size())) {
                    // std::cout << " timestep: " << estimator_IEKF.CSVData.timestamp[csv_index];
                }
            }
            std::cout << " position: "
                      << estimator_IEKF.robot_state.Position.transpose()
                      << std::endl;

            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            step_times.push_back(elapsed.count());

        }
        // std::cout << "Final robot state:" << std::endl;
        // std::cout << "  Position: " << estimator_IEKF.robot_state.Position.transpose() << std::endl;
        // std::cout << "  Velocity: " << estimator_IEKF.robot_state.Velocity.transpose() << std::endl;
        // std::cout << "  Orientation: " << estimator_IEKF.robot_state.Rotation << std::endl;
        // std::cout << "  Bias Gyro: " << std::scientific << std::setprecision(6) 
        //           << estimator_IEKF.robot_state.Bias_Gyro.transpose() << std::endl;
        // std::cout << "  Bias Acc: " << std::scientific << std::setprecision(6) 
        //           << estimator_IEKF.robot_state.Bias_Acc.transpose() << std::endl;
        // if (estimator_IEKF.csv_mode && !estimator_IEKF.CSVData.pos_x.empty() && !estimator_IEKF.CSVData.timestamp.empty()) {
        //     const int last_gt_index = estimator_IEKF.time_length - 1;
        //     Eigen::Vector3d gt_position;
        //     gt_position << estimator_IEKF.CSVData.pos_x[last_gt_index],
        //                    estimator_IEKF.CSVData.pos_y[last_gt_index],
        //                    estimator_IEKF.CSVData.pos_z[last_gt_index];
        //     std::cout << "  Ground truth position: " << gt_position.transpose() << std::endl;
        //     std::cout << "  Time index: " << estimator_IEKF.CSVData.timestamp[last_gt_index] << std::endl;
        // } else {
        //     std::cout << "  Ground truth position: unavailable" << std::endl;
        //     std::cout << "  Time index: unavailable" << std::endl;
        // }
        
        // Compute and print timing statistics
        double total_time = 0.0;
        double max_time = 0.0;
        for (int i = 0; i < step_times.size(); i++) {
            total_time += step_times[i];
            if (step_times[i] > max_time) {
                max_time = step_times[i];
            }
        }
        double avg_time = total_time / step_times.size();
        
        std::cout << "Timing Statistics:" << std::endl;
        std::cout << "  Average time per step: " << std::fixed 
                  << std::setprecision(8) << avg_time << " s" << std::endl;
        std::cout << "  Max time per step:     " << std::fixed 
                  << std::setprecision(8) << max_time << " s" << std::endl;
        std::cout << "  Total time:            " << std::fixed 
                  << std::setprecision(8) << total_time << " s" << std::endl;

        InEKF_SaveUtils::print_contact_accuracy_impl(estimator_IEKF);

        if (estimator_IEKF.save_results) {
            InEKF_SaveUtils::write_saved_data_impl(estimator_IEKF);
        } else {
            cout << "Skipping result saving (save_results = false)" << endl;
        }

    return 0;
}
