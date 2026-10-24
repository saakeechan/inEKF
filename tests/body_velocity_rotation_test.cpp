// Standalone regression: link with src/InEKF/{InEKF,RobotState,NoiseParams,LieGroup}.cpp.
#include "src/InEKF/InEKF.h"
#include <cassert>
#include <iostream>

int main() {
    for (int dimX : {5, 7}) {
        const int dimP = 3 * dimX;
        const int bias = dimP - 6;
        Eigen::MatrixXd X = Eigen::MatrixXd::Identity(dimX, dimX);
        X.topLeftCorner<3,3>() = inekf::Exp_SO3(Eigen::Vector3d(0.2, -0.1, 0.3));
        X.block<3,1>(0,3) = Eigen::Vector3d(0.1, -0.2, 0.05);
        Eigen::VectorXd theta = Eigen::VectorXd::Constant(6, 0.01);
        Eigen::MatrixXd P = Eigen::MatrixXd::Identity(dimP, dimP);
        // Nonzero correlations exercise every route from velocity to attitude/bias.
        P.block<3,3>(0,3) = 0.2 * Eigen::Matrix3d::Identity();
        P.block<3,3>(3,0) = P.block<3,3>(0,3).transpose();
        P.block<3,3>(bias,3) = 0.15 * Eigen::Matrix3d::Identity();
        P.block<3,3>(3,bias) = P.block<3,3>(bias,3).transpose();
        P.block<3,3>(0,bias) = 0.05 * Eigen::Matrix3d::Identity();
        P.block<3,3>(bias,0) = P.block<3,3>(0,bias).transpose();
        P.block<3,3>(bias+3,3) = 0.1 * Eigen::Matrix3d::Identity();
        P.block<3,3>(3,bias+3) = P.block<3,3>(bias+3,3).transpose();

        inekf::RobotState initial(X, theta, P);
        inekf::InEKF filter(initial), original(initial);
        const Eigen::Vector3d velocity(0.8, -0.3, 0.2);
        const Eigen::Vector3d variance(0.01, 0.2, 0.7);
        filter.CorrectBodyVelocity(velocity, variance);
        auto result = filter.getState();
        const Eigen::MatrixXd updatedP = result.getP();
        assert(result.getRotation().isApprox(initial.getRotation(), 1e-12));
        assert(result.getGyroscopeBias().isApprox(initial.getGyroscopeBias(), 1e-12));
        assert(!result.getAccelerometerBias().isApprox(initial.getAccelerometerBias(), 1e-8));
        assert((result.getRotation()*velocity-result.getVelocity()).norm()
             < (initial.getRotation()*velocity-initial.getVelocity()).norm());
        assert(updatedP.topLeftCorner(3,3).isApprox(P.topLeftCorner(3,3), 1e-12));
        assert(updatedP.block(bias,bias,3,3).isApprox(P.block(bias,bias,3,3), 1e-12));
        assert(updatedP.block(0,bias,3,3).isApprox(P.block(0,bias,3,3), 1e-12));
        assert(updatedP.block(bias,0,3,3).isApprox(P.block(bias,0,3,3), 1e-12));
        assert(!updatedP.block(0,3,3,3).isApprox(P.block(0,3,3,3), 1e-8));
        assert(!updatedP.block(bias,3,3,3).isApprox(P.block(bias,3,3,3), 1e-8));
        assert(updatedP.isApprox(updatedP.transpose(), 1e-12));
        assert(Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(updatedP).eigenvalues().minCoeff() > 0);

        // The original correction still permits rotation and gyro-bias updates.
        Eigen::VectorXd Y = Eigen::VectorXd::Zero(dimX), b = Y;
        Y.head<3>() = velocity;
        Y(3) = b(3) = -1;
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, dimP);
        H.block<3,3>(0,3).setIdentity();
        Eigen::MatrixXd PI = Eigen::MatrixXd::Zero(3, dimX);
        PI.leftCols<3>().setIdentity();
        const Eigen::Matrix3d R = initial.getRotation();
        Eigen::MatrixXd N = R * variance.asDiagonal() * R.transpose();
        original.Correct(inekf::Observation(Y, b, H, N, PI));
        auto unrestricted = original.getState();
        assert(!unrestricted.getRotation().isApprox(initial.getRotation(), 1e-8));
        assert(!unrestricted.getGyroscopeBias().isApprox(initial.getGyroscopeBias(), 1e-8));
    }
    std::cout << "Body velocity preserves rotation/gyro bias and their covariance blocks; "
                 "velocity/accel bias update, and original Correct remains unrestricted.\n";
}
