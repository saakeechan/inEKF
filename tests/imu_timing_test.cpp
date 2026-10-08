// Run from build/ after linking against INEKF_Estimator_x64.
#include "src/estimatorManager/InEKF_manager.hpp"
#include <cassert>
#include <iostream>

int main() {
    InEKFilter estimator("../config/initialization_config.yaml", "../config/G1_config.yaml");
    Eigen::Vector<double, 12> covariance = Eigen::Vector<double, 12>::Constant(1e-6);
    Eigen::Vector<double, 16> initial = Eigen::Vector<double, 16>::Zero();
    initial(3) = 1.0;
    estimator.save_results = false;

    for (int n : {2, 3, 5}) {
        estimator.csv_mode = true;
        estimator.Initialize(0.02, covariance, initial, n);
        Eigen::VectorXd previous = estimator.ReadCSVIMU(estimator.start_index);
        assert(estimator.IMU_Gyro[n - 2].isApprox(previous.head<3>()));
        assert(estimator.IMU_Acc[n - 2].isApprox(previous.tail<3>()));
        for (int step = 1; step <= 3; ++step) {
            const Eigen::VectorXd fresh = estimator.ReadCSVIMU(estimator.start_index + step);
            estimator.RunCSVStep();
            assert(estimator.imu_measurement.isApprox(previous));
            assert(estimator.IMU_Gyro[n - 2].isApprox(fresh.head<3>()));
            assert(estimator.IMU_Acc[n - 2].isApprox(fresh.tail<3>()));
            previous = fresh;
        }

        // Exercise both live overloads with distinct samples and verify startup/reset.
        estimator.csv_mode = false;
        for (bool with_contacts : {false, true}) {
            estimator.Initialize(0.02, covariance, initial, n);
            Eigen::MatrixXd motors = Eigen::MatrixXd::Zero(2, 29);
            const std::vector<bool> contacts(estimator.robot.num_legs, false);
            for (int step = 1; step <= 3; ++step) {
                Eigen::VectorXd fresh(6);
                fresh << 0.01 * step, 0.02 * step, 0.03 * step,
                         0.1 * step, 0.2 * step, 9.806659;
                if (step == 1) previous = fresh; // First live sample seeds history.
                auto expected = estimator.filter;
                expected.Propagate(previous, estimator.dt, estimator.Variable_Contact_Cov(n - 2));
                if (with_contacts) estimator.Onestep(fresh, contacts, motors);
                else estimator.Onestep(fresh);
                assert(estimator.robot_state.Rotation.isApprox(expected.getState().getRotation()));
                assert(estimator.robot_state.Position.isApprox(expected.getState().getPosition()));
                assert(estimator.robot_state.Velocity.isApprox(expected.getState().getVelocity()));
                assert(estimator.imu_measurement.isApprox(previous));
                assert(estimator.IMU_Gyro[n - 2].isApprox(fresh.head<3>()));
                assert(estimator.IMU_Acc[n - 2].isApprox(fresh.tail<3>()));
                assert(estimator.robot_state.Angular_Velocity.isApprox(
                    fresh.head<3>() - estimator.robot_state.Bias_Gyro));
                previous = fresh;
            }
        }
    }
    std::cout << "Previous IMU propagation passed for CSV and both live overloads, N=2, 3, 5.\n";
}
