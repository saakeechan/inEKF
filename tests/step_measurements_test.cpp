// Run from build/ after linking against INEKF_Estimator_x64.
#include "src/estimatorManager/InEKF_manager.hpp"
#include <cassert>
#include <iostream>
#include <limits>

int main() {
    INEKFilter_manager manager;
    manager.load_robot_config("../config/G1_config.yaml");
    const Eigen::Vector<double,12> covariance = Eigen::Vector<double,12>::Constant(1e-3);
    Eigen::Vector<double,16> initial = Eigen::Vector<double,16>::Zero();
    initial(3) = 1;
    const KinematicsMeasurement kinematics{
        std::vector<bool>(manager.robot.model.num_legs, true), Eigen::MatrixXd::Zero(2, 29)};
    const BodyVelocityMeasurement body{Eigen::Vector3d(0.8, -0.3, 0.2), Eigen::Vector3d(0.01, 0.2, 0.7)};

    for (int n : {2, 3, 5}) for (int combination = 0; combination < 4; ++combination) {
        manager.Initialize(0.02, covariance, initial, n);
        Eigen::VectorXd previous(6);
        previous << 0.1, -0.2, 0.3, 0.2, 0.1, 9.806659;
        // Seed real contact states so the next kinematics correction is active.
        manager.Onestep(previous, StepMeasurements{kinematics});
        assert(manager.filter.getEstimatedContactPositions().size() == 2);
        for (int step = 1; step <= 3; ++step) {
            StepMeasurements measurements;
            if (combination & 1) measurements.kinematics = kinematics;
            if (combination & 2) measurements.body_velocity = body;
            Eigen::VectorXd fresh = previous;
            fresh.head<3>() += Eigen::Vector3d(0.01, 0.02, 0.03);
            fresh.tail<3>() += Eigen::Vector3d(0.1, -0.1, 0.05);
            auto expected = manager.filter;
            expected.Propagate(previous, manager.dt, manager.Variable_Contact_Cov(n - 2));
            if (measurements.kinematics) {
                inekf::vectorKinematics observations;
                std::vector<std::pair<int,bool>> contacts;
                for (int leg = 0; leg < manager.robot.model.num_legs; ++leg) {
                    contacts.emplace_back(leg, true);
                    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
                    pose.topRightCorner<3,1>() = manager.robot.model.get_foot_position(leg);
                    const Eigen::MatrixXd J = manager.robot.model.get_foot_jacobian(leg);
                    Eigen::Matrix<double,6,6> cov = Eigen::Matrix<double,6,6>::Zero();
                    cov.topLeftCorner<3,3>() = manager.cov_enc_const * Eigen::Matrix3d::Identity();
                    cov.bottomRightCorner<3,3>() = manager.cov_enc_const * J * J.transpose();
                    observations.emplace_back(leg, pose, cov);
                }
                expected.setContacts(contacts);
                expected.CorrectKinematics("pkm", observations, true);
            }
            if (measurements.body_velocity) expected.CorrectBodyVelocity(body.velocity, body.variance);
            manager.Onestep(fresh, measurements);
            auto actual = manager.filter.getState();
            auto target = expected.getState();
            assert(actual.getX().isApprox(target.getX(), 1e-10));
            assert(actual.getP().isApprox(target.getP(), 1e-10));
            assert(actual.getTheta().isApprox(target.getTheta(), 1e-10));
            assert(manager.robot.state.Velocity.isApprox(actual.getVelocity()));
            assert(manager.robot.state.Covariance.isApprox(actual.getP()));
            assert(manager.Velocity_s[n-2].isApprox(actual.getVelocity()));
            assert(manager.Rotation_s[n-2].isApprox(actual.getRotation()));
            assert(manager.imu_measurement.isApprox(previous));
            assert(manager.IMU_Gyro[n-2].isApprox(fresh.head<3>()));
            assert(manager.IMU_Acc[n-2].isApprox(fresh.tail<3>()));
            assert(manager.IMU_Gyro.back().isZero() && manager.frame_count == n-1);
            if (measurements.kinematics) {
                for (int leg=0; leg<manager.robot.model.num_legs; ++leg) {
                    const Eigen::Vector3d residual = actual.getVelocity()
                        + actual.getRotation() * manager.robot.model.get_foot_velocity(leg)
                        + actual.getRotation() * inekf::skew(fresh.head<3>()-actual.getGyroscopeBias())
                          * manager.robot.model.get_foot_position(leg);
                    assert(manager.CONTACT_s[n-2].d_v.segment<3>(3*leg).isApprox(residual));
                    assert(std::abs(manager.CONTACT_s[n-2].dv_norm(leg)-residual.norm()) < 1e-10);
                }
            } else {
                assert(manager.CONTACT_s[n-2].d_v.array().isNaN().all());
                assert(manager.CONTACT_s[n-2].dv_norm.array().isNaN().all());
            }
            previous = fresh;
        }
        // Invalid optional measurements must fail before propagation or shifting.
        for (int bad = 0; bad < 6; ++bad) {
            StepMeasurements measurements{kinematics, body};
            if (bad == 0) measurements.body_velocity->variance.x() = 0;
            if (bad == 1) measurements.body_velocity->variance.y() = -1;
            if (bad == 2) measurements.body_velocity->velocity.z() = std::numeric_limits<double>::quiet_NaN();
            if (bad == 3) measurements.kinematics->motor_data.resize(1, 29);
            if (bad == 4) measurements.kinematics->motor_data.resize(2, 28);
            if (bad == 5) measurements.kinematics->contacts.resize(1);
            auto before = manager.filter.getState();
            bool rejected = false;
            try { manager.Onestep(previous, measurements); }
            catch (const std::invalid_argument&) { rejected = true; }
            assert(rejected);
            assert(manager.filter.getState().getX().isApprox(before.getX()));
            assert(manager.filter.getState().getP().isApprox(before.getP()));
            assert(manager.IMU_Gyro[n-2].isApprox(previous.head<3>()));
            assert(manager.IMU_Gyro.back().isZero());
        }
    }
    std::cout << "All four step combinations passed for N=2/3/5: one propagation/shift, sequential corrections, final diagnostics, and input rejection.\n";
}
