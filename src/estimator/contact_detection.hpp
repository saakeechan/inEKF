/* ----------------------------------------------------------------------------
 * Contact Detection Module
 * Standalone contact detection logic independent of filter implementation
 * -------------------------------------------------------------------------- */

#ifndef CONTACT_DETECTION_HPP
#define CONTACT_DETECTION_HPP

#include <vector>
#include <eigen3/Eigen/Dense>

namespace ContactDetection {

    // --- Velocity detection parameters ---
    inline constexpr double dv0          = 0.3;   // midpoint: p = 0.5

    inline constexpr double dv_contact   = 1;  // confident contact below dv0
    inline constexpr double p_contact    = 0.95;

    inline constexpr double dv_nocontact = 0.7;   // confident NO contact above dv0
    inline constexpr double p_nocontact  = 0.05;

    // --- Torque detection parameters ---
    inline constexpr double T0 = 800.0;        // midpoint: p = 0.5

    inline constexpr double T_high = 2000.0;  // confident contact (high torque)
    inline constexpr double p_high = 0.95;
    inline constexpr double T_low  = 50.0;    // confident NO contact (low torque)
    inline constexpr double p_low  = 0.05;

    // --- Fusion weights ---
    inline constexpr double w_torque   = 0.6;
    inline constexpr double w_velocity = 0.4;
    
    // ---------------------------------------------------------


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
        const Eigen::VectorXd& leg_speeds // Optional: could be used for additional logic
    );

    /**
     * @brief Detect ground contact using only torque information
     * 
     * @param leg_torques Vector of torque values for each leg
     * @return std::vector<double> Contact probability for each leg (0.0 to 1.0)
     */
    std::vector<double> detect_contact_torque(
        const Eigen::VectorXd& leg_torques
    );

    /**
     * @brief Detect ground contact using only velocity residual information
     * 
     * @param d_v Foot velocity residuals (size: 3*num_legs, ordered as [leg1_xyz, leg2_xyz, ...])
     * @return std::vector<double> Contact probability for each leg (0.0 to 1.0)
     */
    std::vector<double> detect_contact_velocity(
        const Eigen::VectorXd& d_v
    );

} // namespace ContactDetection

#endif // CONTACT_DETECTION_HPP
