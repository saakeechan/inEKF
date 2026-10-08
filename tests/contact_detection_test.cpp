#include "src/contactDetection/contact_detection.hpp"
#include <cassert>
#include <stdexcept>

int main() {
    const std::vector<std::vector<int>> legs{{0, 2}, {1, 3}};
    Eigen::Vector4d torques(30.0, 1.0, -40.0, -2.0);
    Eigen::Vector4d velocities(-1.0, 1.0, 0.5, -0.5);
    Eigen::VectorXd residual = Eigen::VectorXd::Zero(6);
    const auto contacts = ContactDetection::get_contacts(torques, velocities, residual, legs);
    assert(contacts == std::vector<bool>({true, false}));

    // Standing overrides torque probabilities, as in the existing algorithm.
    velocities.setZero();
    assert(ContactDetection::get_contacts(torques, velocities, residual, legs)
           == std::vector<bool>({true, true}));

    // Reject an invalid joint map before indexing the motor frame.
    bool rejected = false;
    try {
        ContactDetection::get_contacts(torques, velocities, residual, {{4}});
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    assert(rejected);
}
