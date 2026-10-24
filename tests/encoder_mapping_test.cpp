#include "src/utility/RobotModel.hpp"
#include <cassert>

int main() {
    RobotModel robot;
    robot.encoder_size = 5;
    robot.encoderdot_size = 7;
    robot.Encoder_indices_map = {3, 1};
    robot.Encoderdot_indices_map = {6, 2};
    const std::vector<double> measurements = {10.0, 20.0};

    Eigen::VectorXd positions = robot.map_indices_to_encoders(measurements, "encoder");
    Eigen::VectorXd velocities = robot.map_indices_to_encoders(measurements, "encoderdot");
    Eigen::VectorXd expected_positions(5);
    expected_positions << 0.0, 20.0, 0.0, 10.0, 0.0;
    Eigen::VectorXd expected_velocities(7);
    expected_velocities << 0.0, 0.0, 20.0, 0.0, 0.0, 0.0, 10.0;
    assert(positions.isApprox(expected_positions));
    assert(velocities.isApprox(expected_velocities));
}
