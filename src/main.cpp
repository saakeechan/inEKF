
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

InEKFilter estimator_IEKF;

/// @brief 
/// @return 
int main() {

    // ------------------------------------------------------------
    // USER CONFIGURABLE PARAMETERS

    std::string robot_config_path = "../config/digit_config.yaml";

    int SLIDING_WINDOW_SIZE = 2;

    double dt = 0.005;


    // TextFile data file path and length
    bool textfileFlag = true; // Set to true to use TextFile data, false to use sensor data

    std::string file_name = "../digit_cropped_data.csv";
    int time_length = 976; // total number of data points to process from the TextFile file
    int TextFile_Starting_Point = 0; // number of initial data points to skip in TextFile

    bool SR = true; // slip rejection mode
    double slip_thr = 0.3; // slip threshold in m/s
    bool VCC = false; // variable contact covariance mode 
    double cov_amplifier = 5; // this is multiplied to contact covariance when VCC is true


    // Slip covariance is the covariance used when we detect slip and we blow up the contact covariance
    // Contact covariance is the nominal contact covariance

    Eigen::Vector<double, 12> Estimator_Covariances; // Initialize Estimator Covariances which is a 12x1 vector
    //gyro, acc, slip, contact, encoder, bias gyro, bias acc,
    //prior ori, prior vel, prior pos, prior bias gyro, prior bias acc

    double gyro_exp = -5, acc_exp = -1, slip_exp = -1, contact_exp = -4, encoder_exp = -8; // for paper
    double bg_exp = -10, ba_exp = -7;
    double pri_ori_exp = -10, pri_vel_exp = -10, pri_pos_exp = -10;
    double pri_bg_exp = -10, pri_ba_exp = -10;

    Eigen::Vector<double, 16> initial_condition;
    initial_condition << 0.0, 0.0, 0.0, // Px,Py,Pz
            0.0, 0.0, 0.0, 0.0, // (indices 3 4 5 correspond to roll pitch yaw)
            0.0, 0.0, 0.0,     //Vx, Vy, Vz
            0.0, 0.0, 0.0,     //bgx, bgy, bgz
            0.0, 0.0, 0.0;    //bax, bay, baz


    // ------------------------------------------------------------



    Estimator_Covariances << pow(10, gyro_exp), pow(10, acc_exp), pow(10, slip_exp), pow(10, contact_exp), pow(10,encoder_exp), //gyro, acc, contact, encoder
    pow(10, bg_exp), pow(10, ba_exp), //bias gyro, bias acc
    pow(10, pri_ori_exp), pow(10, pri_vel_exp), pow(10, pri_pos_exp), //prior ori, vel, pos, contact
    pow(10, pri_bg_exp), pow(10, pri_ba_exp); //prior bias gyro, acc

    std::string cov_info; // Do we really need this string? FIX STRUCTURE BADLY
    cov_info = "(" + BasicFunctions_Estimator::to_string_n_signficant_figures(gyro_exp, 2) +
               BasicFunctions_Estimator::to_string_n_signficant_figures(acc_exp, 2) +
               BasicFunctions_Estimator::to_string_n_signficant_figures(slip_exp, 2) +
               BasicFunctions_Estimator::to_string_n_signficant_figures(contact_exp, 2) +
               BasicFunctions_Estimator::to_string_n_signficant_figures(encoder_exp, 2)
               + " " + BasicFunctions_Estimator::to_string_n_signficant_figures(bg_exp, 2)
               + " " + BasicFunctions_Estimator::to_string_n_signficant_figures(pri_ori_exp, 2) +
               BasicFunctions_Estimator::to_string_n_signficant_figures(pri_bg_exp, 2) + ")";


    estimator_IEKF.load_robot_config(robot_config_path); // Load robot configuration
    std::cout << "Robot model loaded successfully." << std::endl;

    Eigen::MatrixXd Sensor_; //
    Sensor_.setZero();
    std::vector<bool> Contact_(estimator_IEKF.robot.num_legs, false); // Initialize Contact_ - dynamic size based on robot config
    ROBOT_STATES state_;

    
    estimator_IEKF.slip_rejection_mode = SR;
    estimator_IEKF.slip_threshold = slip_thr; // This is slip threshold number. Is it in m/s? Find units
    estimator_IEKF.variable_contact_cov_mode = VCC;
    estimator_IEKF.cov_amplifier = cov_amplifier;

    if (textfileFlag) {
        estimator_IEKF.LoadTextFileData(file_name); // Load data BEFORE Initialize so initial state can read from it
        estimator_IEKF.TextFile_Skip_Data = TextFile_Starting_Point;
    }

    estimator_IEKF.Initialize(dt, Estimator_Covariances, initial_condition, SLIDING_WINDOW_SIZE);

    std::vector<double> step_times;

    if (textfileFlag) {

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

        estimator_IEKF.writeSavedData(cov_info);
        cout << "Reached after estimator_IEKF.writeSavedData(cov_info)" << endl;
    }
    else {
        // ROS2 Integration Mode - Real-time sensor processing
        std::cout << "Starting ROS2 node for real-time estimation..." << std::endl;
        
        // TODO: Initialize ROS2 node
        // rclcpp::init(argc, argv);
        // auto node = std::make_shared<rclcpp::Node>("inekf_estimator_node");
        
        // TODO: Create subscribers for sensor data
        // 1. IMU subscriber (for gyro and accelerometer)
        // auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        //     "/imu/data", 10, 
        //     [&](const sensor_msgs::msg::Imu::SharedPtr msg) {
        //         // Extract gyro: indices 0-2 in Sensor_
        //         Sensor_[0] = msg->angular_velocity.x;
        //         Sensor_[1] = msg->angular_velocity.y;
        //         Sensor_[2] = msg->angular_velocity.z;
        //         // Extract accel: indices 3-5 in Sensor_
        //         Sensor_[3] = msg->linear_acceleration.x;
        //         Sensor_[4] = msg->linear_acceleration.y;
        //         Sensor_[5] = msg->linear_acceleration.z;
        //     });
        
        // 2. Joint states subscriber (for encoders)
        // auto joint_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        //     "/joint_states", 10,
        //     [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
        //         // Map joint positions to Sensor_[6:6+n] based on robot config
        //         // Map joint velocities to appropriate indices
        //         // Use estimator_IEKF.robot.textfile_encoder_pos_mapping
        //     });
        
        // 3. Contact state subscriber
        // auto contact_sub = node->create_subscription<std_msgs::msg::BoolMultiArray>(
        //     "/contact_states", 10,
        //     [&](const std_msgs::msg::BoolMultiArray::SharedPtr msg) {
        //         for (size_t i = 0; i < Contact_.size() && i < msg->data.size(); i++) {
        //             Contact_[i] = msg->data[i];
        //         }
        //     });
        
        // TODO: Create publisher for estimated state
        // auto state_pub = node->create_publisher<nav_msgs::msg::Odometry>(
        //     "/inekf/odometry", 10);
        
        // TODO: Create timer for periodic estimation (if needed)
        // auto timer = node->create_wall_timer(
        //     std::chrono::milliseconds(int(dt * 1000)),
        //     [&]() {
        //         // Run one estimation step
        //         estimator_IEKF.Onestep(Sensor_, Contact_, state_);
        //         
        //         // Publish estimated state
        //         auto odom_msg = nav_msgs::msg::Odometry();
        //         odom_msg.header.stamp = node->now();
        //         odom_msg.header.frame_id = "world";
        //         odom_msg.child_frame_id = "base_link";
        //         
        //         // Fill in position from state_.X (rotation matrix and position)
        //         // odom_msg.pose.pose.position.x = state_.X(0, 4);
        //         // odom_msg.pose.pose.position.y = state_.X(1, 4);
        //         // odom_msg.pose.pose.position.z = state_.X(2, 4);
        //         
        //         // Convert rotation matrix to quaternion
        //         // Eigen::Matrix3d R = state_.X.block<3,3>(0,0);
        //         // Eigen::Quaterniond q(R);
        //         // odom_msg.pose.pose.orientation.x = q.x();
        //         // odom_msg.pose.pose.orientation.y = q.y();
        //         // odom_msg.pose.pose.orientation.z = q.z();
        //         // odom_msg.pose.pose.orientation.w = q.w();
        //         
        //         // Fill in velocity
        //         // odom_msg.twist.twist.linear.x = state_.X(0, 3);
        //         // odom_msg.twist.twist.linear.y = state_.X(1, 3);
        //         // odom_msg.twist.twist.linear.z = state_.X(2, 3);
        //         
        //         // state_pub->publish(odom_msg);
        //     });
        
        // TODO: Spin the node
        // rclcpp::spin(node);
        // rclcpp::shutdown();
        
        std::cout << "ROS mode skeleton implemented. Uncomment and build with ROS2 dependencies to enable." << std::endl;
    }

    return 0;
}




