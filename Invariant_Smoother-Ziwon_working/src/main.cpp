
#include <iostream>
// #include <fstream>
#include <string>
// #include <cstdlib>

#include <cmath> // for std::pow
#include <chrono> // for timing

#include <eigen3/Eigen/Dense>
// #include <eigen3/Eigen/Geometry>
#include <vector>
#include "InEKF_estimator.hpp"
// #include <sstream>
#include <iomanip>


using std::cout;
using std::endl;

InEKFilter estimator_IEKF;

/// @brief 
/// @return 
int main() {

    double dt = 0.005;
    int starting_point = 0;
    std::string file_name = "../digit_cropped_data.csv";
    int time_length = 977;
    bool SR = true;
    double slip_thr = 0.3;
    bool VCC = false; // variable contact covariance mode
    double cov_amplifier = 5; // this is multiplied to contact covariance when VCC is true


    Eigen::Matrix<double, 12, 1> Estimator_Covariances; // Initialize Estimator Covariances which is a 12x1 vector
    //gyro, acc, slip, contact, encoder, bias gyro, 
    //bias acc, prior ori, prior vel, prior pos, prior bias gyro, prior bias acc
    // Slip covariance is the covariance used when we detect slip and we blow up the contact covariance
    // Contact covariance is the nominal contact covariance


    double gyro_exp = -5, acc_exp = -1, slip_exp = -1, contact_exp = -4, encoder_exp = -8; // for paper
    double bg_exp = -10, ba_exp = -7;
    double pri_ori_exp = -5, pri_vel_exp = -10, pri_pos_exp = -10;
    double pri_bg_exp = -10, pri_ba_exp = -10;


    Estimator_Covariances << pow(10, gyro_exp), pow(10, acc_exp), pow(10, slip_exp), pow(10, contact_exp), pow(10,encoder_exp), //gyro, acc, contact, encoder
            pow(10, bg_exp), pow(10, ba_exp), //bias gyro, bias acc
            pow(10, pri_ori_exp), pow(10, pri_vel_exp), pow(10, pri_pos_exp), //prior ori, vel, pos, contact
            pow(10, pri_bg_exp), pow(10, pri_ba_exp); //prior bias gyro, acc

    Eigen::Matrix<double, 16, 1> initial_condition;
    initial_condition << 0.0, 0.0, 0.0, // Px,Py,Pz
            0.0, 0.0, 0.0, 0.0, // roll, pitch, yaw, (No meaning) (indices 3 4 5 correspond to roll pitch yaw)
            0.0, 0.0, 0.0,     //Vx, Vy, Vz
            0.0, 0.0, 0.0,     //bgx, bgy, bgz
            0.0, 0.0, 0.0;    //bax, bay, baz

    std::string cov_info;
    cov_info = "(" + BasicFunctions_Estimator::to_string_n_signficant_figures(gyro_exp, 2) +
               BasicFunctions_Estimator::to_string_n_signficant_figures(acc_exp, 2) +
               BasicFunctions_Estimator::to_string_n_signficant_figures(slip_exp, 2) +
               BasicFunctions_Estimator::to_string_n_signficant_figures(contact_exp, 2) +
               BasicFunctions_Estimator::to_string_n_signficant_figures(encoder_exp, 2)
               + " " + BasicFunctions_Estimator::to_string_n_signficant_figures(bg_exp, 2)
               + " " + BasicFunctions_Estimator::to_string_n_signficant_figures(pri_ori_exp, 2) +
               BasicFunctions_Estimator::to_string_n_signficant_figures(pri_bg_exp, 2) + ")";

    // print example cov_info : (5.0-1.0-1.0-4.0-8.0 -10.0 -100.0-10.0)
    // What happened to the other covariances? They are not included in the string
    //
    Eigen::Matrix<double, 30, 1> Sensor_; // Initialize Sensor_ which is a 30x1 vector
    Sensor_.setZero();
    Eigen::Matrix<bool, 2, 1> Contact_; // Initialize Contact_ which is a 2x1 boolean vector covering both legs
    Contact_.setZero();
    ROBOT_STATES state_; // Initialize state_ which is a struct defined in RobotState_Smoother.hpp
    // 3*5 + 3*contact_leg_num(4) + 3*contact_leg_num(4)(vel) = max 39

    estimator_IEKF.robot.leg_no = 2;

    estimator_IEKF.NUM_OF_TRASH_DATA = starting_point;
    estimator_IEKF.LoadCSVData(file_name);
    estimator_IEKF.slip_rejection_mode = SR;
    estimator_IEKF.slip_threshold = slip_thr; // This is slip threshold number. Is it in m/s? Find units
    estimator_IEKF.variable_contact_cov_mode = VCC;
    estimator_IEKF.cov_amplifier = cov_amplifier;

    //robot.define(REAL_ROBOT);
    estimator_IEKF.Initialize(dt, Estimator_Covariances, initial_condition);

    std::vector<double> step_times;
                
    for (int time = 0; time < time_length; time++) {
        auto start = std::chrono::high_resolution_clock::now();

        estimator_IEKF.Onestep(Sensor_, Contact_, state_);

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

    estimator_IEKF.do_SAVE_Z1_all(cov_info);
    cout << "Reached after estimator_IEKF.do_SAVE_Z1_all(cov_info)" << endl;

    return 0;
}




