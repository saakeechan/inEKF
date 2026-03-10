/* ----------------------------------------------------------------------------
 * Contact Detection Module Implementation
 * -------------------------------------------------------------------------- */

#include "contact_detection.hpp"
#include <cmath>
#include <iostream>

namespace ContactDetection {

    std::vector<bool> detect_contact(
        const Eigen::VectorXd& leg_torques,
        const Eigen::VectorXd& d_v,
        double torque_threshold,
        double velocity_threshold
    ) {
        int num_legs = leg_torques.size();
        std::vector<bool> contacts(num_legs, false);
        
        // Validate input sizes
        if (d_v.size() != 3 * num_legs) {
            std::cerr << "Warning: d_v size (" << d_v.size() 
                      << ") doesn't match 3*num_legs (" << 3*num_legs << ")" << std::endl;
            // Fall back to torque-only detection
            std::vector<double> torque_probs = detect_contact_torque(leg_torques, torque_threshold);
            for (int i = 0; i < num_legs; i++) {
                contacts[i] = (torque_probs[i] > 0.5);
            }
            return contacts;
        }
        
        // Get probabilities from both methods
        std::vector<double> torque_probs = detect_contact_torque(leg_torques, torque_threshold);
        std::vector<double> velocity_probs = detect_contact_velocity(d_v, velocity_threshold);
        
        // Fuse probabilities: multiply them (treating as independent probabilities)
        // p(contact | torque AND velocity) ≈ p(torque) * p(velocity) / normalization
        for (int leg = 0; leg < num_legs; leg++) {
            double fused_prob = torque_probs[leg] * velocity_probs[leg];
            contacts[leg] = (fused_prob > 0.25);  // Threshold for fused probability
        }
        
        return contacts;
    }

    std::vector<double> detect_contact_torque(
        const Eigen::VectorXd& leg_torques,
        double torque_threshold
    ) {
        // Asymmetric Torque -> contact probability
        // Parameters
        const double T0 = 5000.0;        // midpoint: p = 0.5
        const double zclip = 10.0;
        
        // Anchors
        const double T_high = 22500.0;   // confident contact (high torque)
        const double p_high = 0.95;
        const double T_low = 200.0;      // confident NO contact (low torque)
        const double p_low = 0.05;
        
        // Distances from T0
        const double d_high = T_high - T0;   // >0
        const double d_low = T0 - T_low;     // >0
        
        // Map anchors through logit: log(p/(1-p))
        const double a_high = std::log(p_high / (1.0 - p_high));  // positive
        const double a_low = std::log(p_low / (1.0 - p_low));     // negative
        
        // Asymmetric quadratic gains
        const double k_high = a_high / (d_high * d_high);  // curvature for T > T0
        const double k_low = (-a_low) / (d_low * d_low);   // curvature for T < T0
        
        // Lambda function for asymmetric torque to probability conversion
        auto torque2prob_asym = [T0, k_high, k_low, zclip](double torque) -> double {
            double d = torque - T0;  // signed distance from midpoint
            double z;
            
            if (d >= 0) {
                // Above T0: want high contact prob => positive z
                z = k_high * (d * d);
            } else {
                // Below T0: want low contact prob => negative z
                z = -k_low * (d * d);
            }
            
            // Clip z
            z = std::max(std::min(z, zclip), -zclip);
            
            // Sigmoid
            double p = 1.0 / (1.0 + std::exp(-z));
            return p;
        };
        
        int num_legs = leg_torques.size();
        std::vector<double> contact_probabilities(num_legs, 0.0);
        
        for (int leg = 0; leg < num_legs; leg++) {
            double torque = std::abs(leg_torques(leg));
            contact_probabilities[leg] = torque2prob_asym(torque);
        }
        
        return contact_probabilities;
    }

    std::vector<double> detect_contact_velocity(
        const Eigen::VectorXd& d_v,
        double velocity_threshold
    ) {
        // Asymmetric DV -> contact probability (piecewise)
        // Parameters
        const double dv0 = 0.3;        // midpoint: p = 0.5
        const double zclip = 4.0;
        
        // Anchors
        const double dv_contact = 0.10;     // confident contact below dv0
        const double p_contact = 0.95;
        const double dv_nocontact = 1.0;    // confident NO contact above dv0
        const double p_nocontact = 0.05;
        
        // Distances from dv0
        const double dL = dv0 - dv_contact;      // >0
        const double dU = dv_nocontact - dv0;    // >0
        
        // Map anchors through logit: log(p/(1-p))
        const double aL = std::log(p_contact / (1.0 - p_contact));           // positive
        const double aU = std::log(p_nocontact / (1.0 - p_nocontact));       // negative
        
        // Gains
        const double kL = aL / (dL * dL);           // >0
        const double kU = (-aU) / (dU * dU);        // >0
        
        // Lambda function for asymmetric DV to probability conversion
        auto dv2prob_asym = [dv0, kL, kU, zclip](double dv) -> double {
            double d = dv - dv0;
            double z;
            
            if (d <= 0) {
                // Below dv0: want high contact prob => positive z
                z = -kL * (d * std::abs(d));   // d*|d| is negative => z positive
            } else {
                // Above dv0: want low contact prob => negative z
                z = -kU * (d * std::abs(d));   // d*|d| positive => z negative
            }
            
            // Clip z
            z = std::max(std::min(z, zclip), -zclip);
            
            // Sigmoid
            double p = 1.0 / (1.0 + std::exp(-z));
            return p;
        };
        
        // Calculate number of legs
        int num_legs = d_v.size() / 3;
        std::vector<double> contact_probabilities(num_legs, 0.0);
        
        // Process each leg
        for (int leg = 0; leg < num_legs; leg++) {
            // Extract velocity residual norm for this leg
            Eigen::Vector3d leg_dv = d_v.segment(3 * leg, 3);
            double velocity_norm = leg_dv.norm();
            
            // Calculate contact probability
            contact_probabilities[leg] = dv2prob_asym(velocity_norm);
        }
        
        return contact_probabilities;
    }

} // namespace ContactDetection
