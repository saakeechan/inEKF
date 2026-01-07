#include <iostream>
#include <chrono>
#include <iomanip>
#include <eigen3/Eigen/Dense>
#include "src/Z1_estimation/utility/Robot_Model.hpp"

int main() {
    std::cout << "Testing Forward Kinematics timing for Digit robot..." << std::endl;
    
    // Create robot model
    Robot_Model_Digit robot;
    
    // Create dummy configuration vector (61 DOFs for Digit)
    Eigen::VectorXd q = Eigen::VectorXd::Zero(61);
    
    // Assign random values for all 61 DOFs
    q << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,  // Base position (0-2) and quaternion (3-6)
         0.1, 0.05, 0.2, 0.0, 0.0, 0.0, 0.0,  // Left hip (7-9), achilles-rod (10-13)
         -0.4, 0.1, -0.2, 0.05,                // Left knee, shin, tarsus, heel-spring (14-17)
         0.08, 0.0, 0.0, 0.0, 0.0,             // Left toe-A (18), toe-A-rod (19-22)
         0.06, 0.0, 0.0, 0.0, 0.0,             // Left toe-B (23), toe-B-rod (24-27)
         0.03, 0.02,                            // Left toe pitch, roll (28-29)
         0.0, 0.0, 0.0, 0.0,                   // Left arm (30-33)
         -0.1, -0.05, 0.2, 0.0, 0.0, 0.0, 0.0, // Right hip (34-36), achilles-rod (37-40)
         -0.4, 0.1, -0.2, 0.05,                // Right knee, shin, tarsus, heel-spring (41-44)
         0.08, 0.0, 0.0, 0.0, 0.0,             // Right toe-A (45), toe-A-rod (46-49)
         0.06, 0.0, 0.0, 0.0, 0.0,             // Right toe-B (50), toe-B-rod (51-54)
         0.03, 0.02,                            // Right toe pitch, roll (55-56)
         0.0, 0.0, 0.0, 0.0;                   // Right arm (57-60)
    
    std::cout << "\nConfiguration vector has " << q.size() << " elements" << std::endl;
    std::cout << "Running FK timing test..." << std::endl;
    
    // Warm up
    for (int i = 0; i < 10; i++) {
        robot.Forward_Kinematics(q);
    }
    
    // Time multiple iterations
    const int num_iterations = 1000;
    std::vector<double> timings;
    timings.reserve(num_iterations);
    
    for (int i = 0; i < num_iterations; i++) {
        auto start = std::chrono::high_resolution_clock::now();
        robot.Forward_Kinematics(q);
        auto end = std::chrono::high_resolution_clock::now();
        
        std::chrono::duration<double> elapsed = end - start;
        timings.push_back(elapsed.count());
    }
    
    // Calculate statistics
    double total = 0.0;
    double min_time = timings[0];
    double max_time = timings[0];
    
    for (double t : timings) {
        total += t;
        if (t < min_time) min_time = t;
        if (t > max_time) max_time = t;
    }
    
    double avg_time = total / num_iterations;
    
    // Print results
    std::cout << "\n=== FK Timing Results (" << num_iterations << " iterations) ===" << std::endl;
    std::cout << std::fixed << std::setprecision(9);
    std::cout << "Average: " << avg_time << " seconds (" << avg_time * 1000.0 << " ms)" << std::endl;
    std::cout << "Min:     " << min_time << " seconds (" << min_time * 1000.0 << " ms)" << std::endl;
    std::cout << "Max:     " << max_time << " seconds (" << max_time * 1000.0 << " ms)" << std::endl;
    std::cout << "Total:   " << total << " seconds" << std::endl;
    
    // Test that FK actually computed something
    std::cout << "\n=== Verifying FK output ===" << std::endl;
    robot.Forward_Kinematics(q);
    
    Eigen::Vector3d left_foot_pos = robot.get_foot_position(1);
    Eigen::Vector3d right_foot_pos = robot.get_foot_position(2);
    
    std::cout << "Left foot position:  [" << left_foot_pos.transpose() << "]" << std::endl;
    std::cout << "Right foot position: [" << right_foot_pos.transpose() << "]" << std::endl;
    
    return 0;
}
