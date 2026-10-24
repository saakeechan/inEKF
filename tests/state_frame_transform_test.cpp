#include "src/utility/RobotModel.hpp"
#include <pinocchio/multibody/joint/joint-revolute.hpp>
#include <stdexcept>

int main() {
    RobotModel robot;
    auto& model = robot.pin_model;
    const auto imu_joint = model.addJoint(0, pinocchio::JointModelRZ(),
        pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.2, -0.1, 0.3)), "imu_joint");
    const auto state_joint = model.addJoint(0, pinocchio::JointModelRZ(),
        pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-0.1, 0.4, 0.2)), "state_joint");
    const Eigen::Matrix3d mounting = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()).toRotationMatrix();
    robot.imu_frame_id = model.addFrame(pinocchio::Frame("imu", imu_joint, 0,
        pinocchio::SE3(mounting, Eigen::Vector3d(0.8, 0.2, 0.1)), pinocchio::OP_FRAME));
    const int articulated = model.addFrame(pinocchio::Frame("state", state_joint, 0,
        pinocchio::SE3(mounting.transpose(), Eigen::Vector3d(0.3, -1.1, 0.2)), pinocchio::OP_FRAME));
    const int rigid = model.addFrame(pinocchio::Frame("rigid", imu_joint, 0,
        pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(-0.4, 0.1, 0.2)), pinocchio::OP_FRAME));
    robot.pin_data = pinocchio::Data(model);
    Eigen::Vector2d q(0.7, -1.1), qdot(0.9, -0.6);
    const Eigen::Matrix3d R_WO = Eigen::AngleAxisd(0.6, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
    const Eigen::Vector3d p_WO(1, 2, 3), v_WO(0.2, -0.4, 0.1), w_WO(0.3, -0.2, 0.7);
    auto update = [&](const Eigen::Vector2d& angles) {
        pinocchio::forwardKinematics(model, robot.pin_data, angles, qdot);
        pinocchio::updateFramePlacements(model, robot.pin_data);
    };
    update(q);
    const pinocchio::SE3 imu = robot.pin_data.oMf[robot.imu_frame_id];
    const auto imu_v = pinocchio::getFrameVelocity(model, robot.pin_data,
        robot.imu_frame_id, pinocchio::LOCAL_WORLD_ALIGNED);
    const Eigen::Matrix3d R_WI = R_WO * imu.rotation();
    const Eigen::Vector3d p_WI = p_WO + R_WO * imu.translation();
    const Eigen::Vector3d v_WI = v_WO + w_WO.cross(R_WO * imu.translation()) + R_WO * imu_v.linear();
    const Eigen::Vector3d w_WI = w_WO + R_WO * imu_v.angular();
    constexpr double dt = 1e-6;
    for (int frame : {0, robot.imu_frame_id, rigid, articulated}) {
        update(q);
        Eigen::Matrix3d rotation;
        Eigen::Vector3d position, velocity, angular;
        robot.transform_state_to_frame(frame, R_WI, p_WI, v_WI, w_WI,
            rotation, position, velocity, angular);
        const auto nominal = robot.pin_data.oMf[frame];
        if (!rotation.isApprox(R_WO * nominal.rotation(), 1e-12) ||
            !position.isApprox(p_WO + R_WO * nominal.translation(), 1e-12)) {
            throw std::runtime_error("Incorrect selected-frame pose");
        }
        auto world_pose = [&](double t) {
            update(q + t * qdot);
            const Eigen::Matrix3d R = Eigen::AngleAxisd(t * w_WO.norm(), w_WO.normalized()).toRotationMatrix() * R_WO;
            return pinocchio::SE3(R * robot.pin_data.oMf[frame].rotation(),
                p_WO + t * v_WO + R * robot.pin_data.oMf[frame].translation());
        };
        const auto plus = world_pose(dt), minus = world_pose(-dt);
        const Eigen::Vector3d numerical_v = (plus.translation() - minus.translation()) / (2 * dt);
        const Eigen::Matrix3d omega_skew = ((plus.rotation() - minus.rotation()) / (2 * dt)) * rotation.transpose();
        const Eigen::Vector3d numerical_w(omega_skew(2,1), omega_skew(0,2), omega_skew(1,0));
        if ((velocity - numerical_v).norm() > 1e-8 || (angular - numerical_w).norm() > 1e-8) {
            throw std::runtime_error("Selected-frame velocities failed finite differences");
        }
    }
    bool rejected = false;
    try {
        Eigen::Matrix3d R;
        Eigen::Vector3d p, v, w;
        robot.transform_state_to_frame(model.nframes, R_WI, p_WI, v_WI, w_WI, R, p, v, w);
    } catch (const std::out_of_range&) { rejected = true; }
    if (!rejected) throw std::runtime_error("Invalid state frame accepted");
}
