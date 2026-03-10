/* ----------------------------------------------------------------------------
 * Contact Detection Module
 * Standalone contact detection logic independent of filter implementation
 * -------------------------------------------------------------------------- */

#ifndef CONTACT_DETECTION_HPP
#define CONTACT_DETECTION_HPP

#include <vector>
#include <eigen3/Eigen/Dense>

namespace ContactDetection {

    /**
     * @brief Detect ground contact combining motor torques and foot velocities
     * 
     * @param leg_torques Vector of torque values for each leg (size: num_legs)
     * @param d_v Foot velocity residuals (size: 3*num_legs, ordered as [leg1_xyz, leg2_xyz, ...])
     * @param torque_threshold Torque magnitude threshold for contact detection (Nm)
     * @param velocity_threshold Velocity threshold for contact validation (m/s)
     * @return std::vector<bool> Contact state for each leg (true = contact, false = swing)
     */
    std::vector<bool> detect_contact(
        const Eigen::VectorXd& leg_torques,
        const Eigen::VectorXd& d_v,
        double torque_threshold = 10.0,
        double velocity_threshold = 0.1
    );

    /**
     * @brief Detect ground contact using only torque information
     * 
     * @param leg_torques Vector of torque values for each leg
     * @param torque_threshold Torque magnitude threshold for contact detection (Nm) - unused, kept for backward compatibility
     * @return std::vector<double> Contact probability for each leg (0.0 to 1.0)
     */
    std::vector<double> detect_contact_torque(
        const Eigen::VectorXd& leg_torques,
        double torque_threshold = 10.0
    );

    /**
     * @brief Detect ground contact using only velocity residual information
     * 
     * @param d_v Foot velocity residuals (size: 3*num_legs, ordered as [leg1_xyz, leg2_xyz, ...])
     * @param velocity_threshold Velocity threshold for contact detection (m/s) - unused, kept for backward compatibility
     * @return std::vector<double> Contact probability for each leg (0.0 to 1.0)
     */
    std::vector<double> detect_contact_velocity(
        const Eigen::VectorXd& d_v,
        double velocity_threshold = 0.1
    );

} // namespace ContactDetection

#endif // CONTACT_DETECTION_HPP
