
#include <iostream>
#include <string>

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
    
    // InEKFilter::getParameters("../src/utility/G1/g1_29dof_floatingBase.urdf", "g1_floating_base");
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

        for (int time = 0; time < (estimator_IEKF.time_length - estimator_IEKF.start_index); time++) {
            auto start = std::chrono::high_resolution_clock::now();

            estimator_IEKF.Onestep();  // Contact detection from torques

            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            step_times.push_back(elapsed.count());

        }
        
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

        InEKF_SaveUtils::write_saved_data_impl(estimator_IEKF);
        cout << "Reached after InEKF_SaveUtils::write_saved_data_impl()" << endl;

    return 0;
}




