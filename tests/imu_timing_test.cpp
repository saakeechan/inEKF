// Run from build/ after linking against INEKF_Estimator_x64.
#include "src/estimatorManager/InEKF_manager.hpp"
#include <cassert>
#include <iostream>

int main() {
    INEKFilter_manager estimator("../config/initialization_config.yaml", "../config/G1_config.yaml");
    Eigen::Vector<double, 12> covariance = Eigen::Vector<double, 12>::Constant(1e-6);
    Eigen::Vector<double, 16> initial = Eigen::Vector<double, 16>::Zero();
    initial(3) = 1.0;
    estimator.save_results = false;

    for (int n : {2, 3, 5}) {
        estimator.csv_mode = true;
        estimator.Initialize(0.02, covariance, initial, n);
        assert(estimator.time_count == 0 && estimator.saved_count == 1);
        assert(estimator.SAVE_MATRIX[estimator.idx_TRUE_Position][0] == estimator.CSVData.pos_x[estimator.start_index]);
        assert(estimator.SAVE_MATRIX[estimator.idx_ESTIMATED_Position][0] == estimator.robot.state.Position.x());
        Eigen::VectorXd previous = estimator.ReadCSVIMU(estimator.start_index);
        assert(estimator.IMU_Gyro[n - 2].isApprox(previous.head<3>()));
        assert(estimator.IMU_Acc[n - 2].isApprox(previous.tail<3>()));
        for (int step = 1; step <= 3; ++step) {
            const Eigen::VectorXd fresh = estimator.ReadCSVIMU(estimator.start_index + step);
            estimator.RunCSVStep();
            assert(estimator.time_count == step && estimator.saved_count == step + 1);
            assert(estimator.SAVE_MATRIX[estimator.idx_TRUE_Position][step] == estimator.CSVData.pos_x[estimator.start_index + step]);
            assert(estimator.robot.state.Covariance.isApprox(estimator.filter.getState().getP()));
            assert(estimator.imu_measurement.isApprox(previous));
            assert(estimator.IMU_Gyro[n - 2].isApprox(fresh.head<3>()));
            assert(estimator.IMU_Acc[n - 2].isApprox(fresh.tail<3>()));
            previous = fresh;
        }

        // Exercise both live measurement modes with distinct samples and verify startup/reset.
        estimator.csv_mode = false;
        auto live_initial = initial;
        live_initial.segment<3>(7) << 0.1, -0.2, 0.3;
        live_initial.segment<3>(10) << 0.01, -0.02, 0.03;
        live_initial.segment<3>(13) << 0.04, -0.05, 0.06;
        for (bool with_contacts : {false, true}) {
            // Stale flags must be cleared when the same manager is reinitialized.
            for (int slot = 0; slot < n; ++slot) {
                estimator.CONTACT_s[slot].contact.assign(estimator.robot.model.num_legs, true);
                estimator.CONTACT_s[slot].hard_contact.assign(estimator.robot.model.num_legs, true);
                estimator.CONTACT_s[slot].slip.assign(estimator.robot.model.num_legs, true);
            }
            estimator.Initialize(0.02, covariance, live_initial, n);
            assert(estimator.filter.getState().getVelocity().isApprox(live_initial.segment<3>(7)));
            assert(estimator.filter.getState().getGyroscopeBias().isApprox(live_initial.segment<3>(10)));
            assert(estimator.filter.getState().getAccelerometerBias().isApprox(live_initial.segment<3>(13)));
            assert(estimator.robot.state.Velocity.isApprox(live_initial.segment<3>(7)));
            assert(estimator.robot.state.Bias_Gyro.isApprox(live_initial.segment<3>(10)));
            assert(estimator.robot.state.Bias_Acc.isApprox(live_initial.segment<3>(13)));
            for (int slot = 0; slot < n; ++slot) {
                const std::vector<bool> cleared(estimator.robot.model.num_legs, false);
                assert(estimator.CONTACT_s[slot].contact == cleared);
                assert(estimator.CONTACT_s[slot].hard_contact == cleared);
                assert(estimator.CONTACT_s[slot].slip == cleared);
            }
            Eigen::MatrixXd motors = Eigen::MatrixXd::Zero(2, 29);
            const std::vector<bool> contacts(estimator.robot.model.num_legs, false);
            for (int step = 1; step <= 3; ++step) {
                Eigen::VectorXd fresh(6);
                fresh << 0.01 * step, 0.02 * step, 0.03 * step,
                         0.1 * step, 0.2 * step, 9.806659;
                if (step == 1) previous = fresh; // First live sample seeds history.
                auto expected = estimator.filter;
                expected.Propagate(previous, estimator.dt, estimator.Variable_Contact_Cov(n - 2));
                if (with_contacts) estimator.Onestep(fresh, StepMeasurements{KinematicsMeasurement{contacts, motors}});
                else estimator.Onestep(fresh);
                assert(estimator.robot.state.Rotation.isApprox(expected.getState().getRotation()));
                assert(estimator.robot.state.Position.isApprox(expected.getState().getPosition()));
                assert(estimator.robot.state.Velocity.isApprox(expected.getState().getVelocity()));
                assert(estimator.robot.state.Covariance.isApprox(estimator.filter.getState().getP()));
                assert(estimator.imu_measurement.isApprox(previous));
                assert(estimator.IMU_Gyro[n - 2].isApprox(fresh.head<3>()));
                assert(estimator.IMU_Acc[n - 2].isApprox(fresh.tail<3>()));
                assert(estimator.robot.state.Angular_Velocity.isApprox(
                    fresh.head<3>() - estimator.robot.state.Bias_Gyro));
                previous = fresh;
            }
        }
    }
    std::cout << "Previous IMU propagation passed for CSV and both live measurement modes, N=2, 3, 5.\n";
}
