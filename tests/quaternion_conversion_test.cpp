#include "../src/utility/BasicFunctions_Estimator.hpp"
#include <limits>
#include <stdexcept>

int main() {
    const Eigen::Vector4d q(0.5, -0.5, 0.5, 0.5);
    const Eigen::Matrix3d reference = BasicFunctions_Estimator::Quaternion_to_Rotation_Matrix(q);
    for (double scale : {1.0, 1.000005278836067, -2.0, 1e-300, 1e308}) {
        const Eigen::Matrix3d rotation = BasicFunctions_Estimator::Quaternion_to_Rotation_Matrix(q * scale);
        if (!rotation.allFinite() || !rotation.isApprox(reference, 1e-12) ||
            !(rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1e-12) ||
            std::abs(rotation.determinant() - 1.0) > 1e-12) {
            throw std::runtime_error("Scaled quaternion did not produce the expected SO(3) matrix");
        }
    }
    for (double invalid : {0.0, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
        Eigen::Vector4d input = Eigen::Vector4d::Zero();
        input(0) = invalid;
        try {
            BasicFunctions_Estimator::Quaternion_to_Rotation_Matrix(input);
        } catch (const std::invalid_argument&) {
            continue;
        }
        throw std::runtime_error("Invalid quaternion was accepted");
    }
}
