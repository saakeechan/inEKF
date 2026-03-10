
#include <iostream>
#include <string>

#include <cmath> // for std::pow
#include <chrono> // for timing

#include <eigen3/Eigen/Dense>
#include <vector>
#include "estimator/InEKF_estimator.hpp"
#include <iomanip>
#include "utility/RobotModel.hpp"


using std::cout;
using std::endl;

/// @brief 
/// @return 
int main() {


    // If this is your first time running a new robot, uncomment the line below
    //  to extract parameters and use that to fill in the yaml files
    
    // InEKFilter::getParameters("/home/saakee/Research/Saaketh_INEKF/src/utility/G1/g1_29dof_IMUShifted.urdf", "g1");
    // return 0;


    // InEKFilter::getParameters("../digit_mujoco-main/src/digit_mujoco/models/flat/digit-v3-flat.xml", "Digit");
    // return 0;

    InEKFilter estimator_IEKF("../config/initialization_config.yaml", "../config/G1_config.yaml");

    std::vector<double> step_times;

        for (int time = 0; time < (estimator_IEKF.time_length - estimator_IEKF.TextFile_Skip_Data); time++) {
            auto start = std::chrono::high_resolution_clock::now();

            estimator_IEKF.Onestep();

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

        estimator_IEKF.writeSavedData();
        cout << "Reached after estimator_IEKF.writeSavedData(cov_info)" << endl;

    return 0;
}




