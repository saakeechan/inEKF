/* ----------------------------------------------------------------------------
 * Dynamic Kinematic Slip Analysis
 * Analyzes base velocity estimates from FK foot positions during contact regions
 * using position-based differentiation and compares against ground truth.
 * Also computes dynamics-based velocity using Pinocchio and zero-velocity constraint.
 * -------------------------------------------------------------------------- */

#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <numeric>
#include <fstream>
#include <iomanip>
#include <eigen3/Eigen/Dense>

// Pinocchio includes
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/constrained-dynamics.hpp>
#include <pinocchio/algorithm/contact-info.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

// Include CSV parser and RobotModel
#include "../src/utility/csvParser.hpp"
#include "../src/utility/RobotModel.hpp"

// Build rotation matrix from quaternion (w, x, y, z)
Eigen::Matrix3d quat_to_rot(double w, double x, double y, double z) {
    Eigen::Quaterniond q(w, x, y, z);
    q.normalize();
    return q.toRotationMatrix();
}

/**
 * @brief Build Pinocchio configuration vector from CSV data at time index i
 * @param data CSV data structure
 * @param i Time index
 * @return Configuration vector q (floating base + joints)
 * 
 * Configuration format for floating base: [x, y, z, qx, qy, qz, qw, joint1, joint2, ...]
 * Floating base: position (3) + quaternion (4) = 7
 * Joints: 29 DOF for G1 robot
 * Total nq = 36
 */
Eigen::VectorXd build_configuration(const DataStruct& data, size_t i) {
    // Configuration space dimension: 7 (floating base) + 29 (joints) = 36
    Eigen::VectorXd q = Eigen::VectorXd::Zero(36);
    
    // Floating base position (x, y, z)
    q[0] = data.pos_x[i];
    q[1] = data.pos_y[i];
    q[2] = data.pos_z[i];
    
    // Floating base orientation (quaternion: x, y, z, w)
    // Pinocchio uses (x, y, z, w) ordering
    q[3] = data.quat_i[i];  // qx
    q[4] = data.quat_j[i];  // qy
    q[5] = data.quat_k[i];  // qz
    q[6] = data.quat_w[i];  // qw
    
    // Joint positions (29 joints for leg+torso, excluding arms/hands)
    // Order matches the URDF joint ordering
    int joint_idx = 7;
    q[joint_idx++] = data.joint_pos_left_hip_pitch[i];
    q[joint_idx++] = data.joint_pos_left_hip_roll[i];
    q[joint_idx++] = data.joint_pos_left_hip_yaw[i];
    q[joint_idx++] = data.joint_pos_left_knee[i];
    q[joint_idx++] = data.joint_pos_left_ankle_pitch[i];
    q[joint_idx++] = data.joint_pos_left_ankle_roll[i];
    
    q[joint_idx++] = data.joint_pos_right_hip_pitch[i];
    q[joint_idx++] = data.joint_pos_right_hip_roll[i];
    q[joint_idx++] = data.joint_pos_right_hip_yaw[i];
    q[joint_idx++] = data.joint_pos_right_knee[i];
    q[joint_idx++] = data.joint_pos_right_ankle_pitch[i];
    q[joint_idx++] = data.joint_pos_right_ankle_roll[i];
    
    q[joint_idx++] = data.joint_pos_waist_yaw[i];
    q[joint_idx++] = data.joint_pos_waist_roll[i];
    q[joint_idx++] = data.joint_pos_waist_pitch[i];
    
    q[joint_idx++] = data.joint_pos_left_shoulder_pitch[i];
    q[joint_idx++] = data.joint_pos_left_shoulder_roll[i];
    q[joint_idx++] = data.joint_pos_left_shoulder_yaw[i];
    q[joint_idx++] = data.joint_pos_left_elbow[i];
    
    q[joint_idx++] = data.joint_pos_right_shoulder_pitch[i];
    q[joint_idx++] = data.joint_pos_right_shoulder_roll[i];
    q[joint_idx++] = data.joint_pos_right_shoulder_yaw[i];
    q[joint_idx++] = data.joint_pos_right_elbow[i];
    
    // Additional joints
    q[joint_idx++] = data.joint_pos_left_wrist_roll[i];
    q[joint_idx++] = data.joint_pos_left_wrist_pitch[i];
    q[joint_idx++] = data.joint_pos_left_wrist_yaw[i];
    q[joint_idx++] = data.joint_pos_right_wrist_roll[i];
    q[joint_idx++] = data.joint_pos_right_wrist_pitch[i];
    q[joint_idx++] = data.joint_pos_right_wrist_yaw[i];
    
    return q;
}

/**
 * @brief Build full velocity vector from CSV data at time index i
 * @param data CSV data structure
 * @param i Time index
 * @param v_base_linear Base linear velocity (3D) - to be filled/solved for
 * @param v_base_angular Base angular velocity (3D) - from gyro or to be solved for
 * @return Full velocity vector: [v_base_linear, v_base_angular, joint_velocities]
 * 
 * Velocity space dimension nv = 6 (floating base) + 29 (joints) = 35
 */
Eigen::VectorXd build_full_velocity(const DataStruct& data, size_t i,
                                     const Eigen::Vector3d& v_base_linear,
                                     const Eigen::Vector3d& v_base_angular) {
    Eigen::VectorXd v = Eigen::VectorXd::Zero(35);
    
    // Floating base velocity (linear + angular)
    v.segment<3>(0) = v_base_linear;
    v.segment<3>(3) = v_base_angular;
    
    // Continue filling joint velocities starting from index 6
    int idx = 6;
    v[idx++] = data.joint_vel_left_hip_pitch[i];
    v[idx++] = data.joint_vel_left_hip_roll[i];
    v[idx++] = data.joint_vel_left_hip_yaw[i];
    v[idx++] = data.joint_vel_left_knee[i];
    v[idx++] = data.joint_vel_left_ankle_pitch[i];
    v[idx++] = data.joint_vel_left_ankle_roll[i];
    
    v[idx++] = data.joint_vel_right_hip_pitch[i];
    v[idx++] = data.joint_vel_right_hip_roll[i];
    v[idx++] = data.joint_vel_right_hip_yaw[i];
    v[idx++] = data.joint_vel_right_knee[i];
    v[idx++] = data.joint_vel_right_ankle_pitch[i];
    v[idx++] = data.joint_vel_right_ankle_roll[i];
    
    v[idx++] = data.joint_vel_waist_yaw[i];
    v[idx++] = data.joint_vel_waist_roll[i];
    v[idx++] = data.joint_vel_waist_pitch[i];
    
    v[idx++] = data.joint_vel_left_shoulder_pitch[i];
    v[idx++] = data.joint_vel_left_shoulder_roll[i];
    v[idx++] = data.joint_vel_left_shoulder_yaw[i];
    v[idx++] = data.joint_vel_left_elbow[i];
    
    v[idx++] = data.joint_vel_right_shoulder_pitch[i];
    v[idx++] = data.joint_vel_right_shoulder_roll[i];
    v[idx++] = data.joint_vel_right_shoulder_yaw[i];
    v[idx++] = data.joint_vel_right_elbow[i];
    
    v[idx++] = data.joint_vel_left_wrist_roll[i];
    v[idx++] = data.joint_vel_left_wrist_pitch[i];
    v[idx++] = data.joint_vel_left_wrist_yaw[i];
    v[idx++] = data.joint_vel_right_wrist_roll[i];
    v[idx++] = data.joint_vel_right_wrist_pitch[i];
    v[idx++] = data.joint_vel_right_wrist_yaw[i];
    
    return v;
}

/**
 * @brief Build joint torque vector from CSV data at time index i
 * @param data CSV data structure
 * @param i Time index
 * @return Joint torque vector (29 DOF)
 */
Eigen::VectorXd build_joint_torques(const DataStruct& data, size_t i) {
    Eigen::VectorXd tau_joints = Eigen::VectorXd::Zero(29);
    
    int idx = 0;
    tau_joints[idx++] = data.joint_torque_left_hip_pitch[i];
    tau_joints[idx++] = data.joint_torque_left_hip_roll[i];
    tau_joints[idx++] = data.joint_torque_left_hip_yaw[i];
    tau_joints[idx++] = data.joint_torque_left_knee[i];
    tau_joints[idx++] = data.joint_torque_left_ankle_pitch[i];
    tau_joints[idx++] = data.joint_torque_left_ankle_roll[i];
    
    tau_joints[idx++] = data.joint_torque_right_hip_pitch[i];
    tau_joints[idx++] = data.joint_torque_right_hip_roll[i];
    tau_joints[idx++] = data.joint_torque_right_hip_yaw[i];
    tau_joints[idx++] = data.joint_torque_right_knee[i];
    tau_joints[idx++] = data.joint_torque_right_ankle_pitch[i];
    tau_joints[idx++] = data.joint_torque_right_ankle_roll[i];
    
    tau_joints[idx++] = data.joint_torque_waist_yaw[i];
    tau_joints[idx++] = data.joint_torque_waist_roll[i];
    tau_joints[idx++] = data.joint_torque_waist_pitch[i];
    
    tau_joints[idx++] = data.joint_torque_left_shoulder_pitch[i];
    tau_joints[idx++] = data.joint_torque_left_shoulder_roll[i];
    tau_joints[idx++] = data.joint_torque_left_shoulder_yaw[i];
    tau_joints[idx++] = data.joint_torque_left_elbow[i];
    
    tau_joints[idx++] = data.joint_torque_right_shoulder_pitch[i];
    tau_joints[idx++] = data.joint_torque_right_shoulder_roll[i];
    tau_joints[idx++] = data.joint_torque_right_shoulder_yaw[i];
    tau_joints[idx++] = data.joint_torque_right_elbow[i];
    
    tau_joints[idx++] = data.joint_torque_left_wrist_roll[i];
    tau_joints[idx++] = data.joint_torque_left_wrist_pitch[i];
    tau_joints[idx++] = data.joint_torque_left_wrist_yaw[i];
    tau_joints[idx++] = data.joint_torque_right_wrist_roll[i];
    tau_joints[idx++] = data.joint_torque_right_wrist_pitch[i];
    tau_joints[idx++] = data.joint_torque_right_wrist_yaw[i];
    
    return tau_joints;
}

/**
 * @brief Kinematic base velocity estimate using foot-fixed Jacobian constraint.
 *
 * Assumes the contact foot has zero velocity and solves:
 *   J_lin * v_base = -(J_ang * omega + J_joints * q_dot)
 *
 * Uses joint positions, joint velocities, base orientation, and gyro.
 * Does NOT use torques.
 */
Eigen::Vector3d compute_base_velocity_kinematics(
    RobotModel& robot,
    const Eigen::VectorXd& q,
    const DataStruct& data,
    size_t i,
    pinocchio::FrameIndex foot_frame_id)
{
    Eigen::Vector3d omega_base(data.gyro_body_x[i], data.gyro_body_y[i], data.gyro_body_z[i]);
    Eigen::VectorXd v = build_full_velocity(data, i, Eigen::Vector3d::Zero(), omega_base);

    // Use robot's FK pipeline (same as RobotModel internals)
    robot.Forward_Kinematics(q, v);

    pinocchio::Motion foot_vel = pinocchio::getFrameVelocity(
        robot.pin_model, robot.pin_data, foot_frame_id, pinocchio::LOCAL_WORLD_ALIGNED);

    // v_base_lin = 0 in v, so foot_vel.linear() = J_ang*omega + J_joints*qdot.
    // Contact constraint (v_foot = 0) => v_base_kin = -foot_vel.linear()
    return -foot_vel.linear();
}

Eigen::Vector3d compute_base_acceleration_dynamics(
    RobotModel& robot,
    const Eigen::VectorXd& q,
    const DataStruct& data,
    size_t i,
    const Eigen::VectorXd& tau_joints,
    const Eigen::Vector3d& v_b_dyn,   // world-frame base linear velocity
    pinocchio::FrameIndex foot_frame_id,
    bool debug = false)
{
    const int nv = robot.pin_model.nv;
    assert(tau_joints.size() == nv - 6);

    // Pinocchio freeflyer: v[0:3] is body-frame linear velocity.
    // Rotate world-frame v_b_dyn to body frame before passing to Pinocchio.
    Eigen::Quaterniond quat_base(q[6], q[3], q[4], q[5]);
    quat_base.normalize();
    Eigen::Matrix3d R_wb = quat_base.toRotationMatrix();

    Eigen::Vector3d omega_base(data.gyro_body_x[i], data.gyro_body_y[i], data.gyro_body_z[i]);
    Eigen::Vector3d v_b_body = R_wb.transpose() * v_b_dyn;
    Eigen::VectorXd v_full = build_full_velocity(data, i, v_b_body, omega_base);

    // Full generalized torque: freeflyer DOFs are unactuated (zero)
    Eigen::VectorXd tau_full = Eigen::VectorXd::Zero(nv);
    tau_full.tail(nv - 6) = tau_joints;

    // 3D point contact at contact foot (enforces zero linear foot velocity)
    typedef pinocchio::RigidConstraintModel ContactModel;
    typedef pinocchio::RigidConstraintData  ContactData;

    std::vector<ContactModel> contact_models = {
        ContactModel(pinocchio::CONTACT_6D, robot.pin_model, foot_frame_id,
                     pinocchio::LOCAL_WORLD_ALIGNED)
    };
    std::vector<ContactData> contact_datas = { ContactData(contact_models[0]) };

    // Allocate internal data structures for the contact problem (lightweight)
    pinocchio::initConstraintDynamics(robot.pin_model, robot.pin_data, contact_models);

    // Solve constrained forward dynamics: fills data.ddq with joint accelerations
    // and contact_datas[0].contact_force with the contact wrench
    pinocchio::constraintDynamics(
        robot.pin_model, robot.pin_data,
        q, v_full, tau_full,
        contact_models, contact_datas);

    // data.ddq[0:3] is body-frame linear acceleration; rotate to world frame
    Eigen::Vector3d ddq_base_body = robot.pin_data.ddq.head<3>();

    if (debug) {
        std::cerr << "  [DYN] tau_joints norm:  " << tau_joints.norm() << "\n"
                  << "  [DYN] ddq_base_body:    " << ddq_base_body.transpose() << "\n"
                  << "  [DYN] |ddq_base|:       " << ddq_base_body.norm() << " m/s²\n"
                  << "  [DYN] contact_force:    "
                  << contact_datas[0].contact_force.linear().transpose() << "\n";
    }

    return R_wb * ddq_base_body;
}

/**
 * @brief Compute ground truth foot velocity from actual simulated foot position
 * @param data Parsed CSV data structure
 * @param lfoot_velocity_world Output: N x 3 matrix of GT foot velocities
 * @param lfoot_velocity_norm Output: N x 1 vector of GT foot velocity norms
 */
void compute_lfoot_velocity_world_GT(const DataStruct& data, 
                                      std::vector<Eigen::Vector3d>& lfoot_velocity_world,
                                      std::vector<double>& lfoot_velocity_norm) {
    size_t N = data.num_rows;
    
    if (N < 2) {
        std::cerr << "Error: Need at least 2 data points" << std::endl;
        return;
    }
    
    lfoot_velocity_world.clear();
    lfoot_velocity_norm.clear();
    lfoot_velocity_world.reserve(N);
    lfoot_velocity_norm.reserve(N);
    
    // Compute numerical differentiation for GROUND TRUTH foot position
    // Use actual simulated foot position from CSV (lfoot_pos_x/y/z)
    for (size_t i = 0; i < N - 1; ++i) {
        Eigen::Vector3d pos_i(data.lfoot_pos_x[i], data.lfoot_pos_y[i], data.lfoot_pos_z[i]);
        Eigen::Vector3d pos_ip1(data.lfoot_pos_x[i+1], data.lfoot_pos_y[i+1], data.lfoot_pos_z[i+1]);

        double dt = data.timestamp[i+1] - data.timestamp[i];
        Eigen::Vector3d velocity = (pos_ip1 - pos_i) / dt;
        
        lfoot_velocity_world.push_back(velocity);
        lfoot_velocity_norm.push_back(velocity.norm() + 1e-8);
    }
    
    // Keep size consistent: duplicate the last velocity
    lfoot_velocity_world.push_back(lfoot_velocity_world.back());
    lfoot_velocity_norm.push_back(lfoot_velocity_norm.back());
}

/**
 * @brief Compute FK-derived foot velocity from base position + FK foot position
 * @param data Parsed CSV data structure
 * @param lfoot_velocity_FK Output: N x 3 matrix of FK-derived foot velocities
 * @param lfoot_velocity_FK_norm Output: N x 1 vector of FK foot velocity norms
 */
void compute_lfoot_velocity_world_FK(const DataStruct& data, 
                                      std::vector<Eigen::Vector3d>& lfoot_velocity_FK,
                                      std::vector<double>& lfoot_velocity_FK_norm) {
    size_t N = data.num_rows;
    
    if (N < 2) {
        std::cerr << "Error: Need at least 2 data points" << std::endl;
        return;
    }
    
    lfoot_velocity_FK.clear();
    lfoot_velocity_FK_norm.clear();
    lfoot_velocity_FK.reserve(N);
    lfoot_velocity_FK_norm.reserve(N);
    
    // Compute numerical differentiation for FK-derived foot position
    // FK foot position in world frame = base_pos + R * fk_foot_pos_base
    for (size_t i = 0; i < N - 1; ++i) {
        // Rotation matrix from quaternion
        Eigen::Matrix3d R_i = quat_to_rot(data.quat_w[i], data.quat_i[i],
                                          data.quat_j[i], data.quat_k[i]);
        Eigen::Matrix3d R_ip1 = quat_to_rot(data.quat_w[i+1], data.quat_i[i+1],
                                            data.quat_j[i+1], data.quat_k[i+1]);
        
        // FK foot position in base frame
        Eigen::Vector3d fk_foot_base_i(data.fk_left_foot_pos_x[i],
                                       data.fk_left_foot_pos_y[i],
                                       data.fk_left_foot_pos_z[i]);
        Eigen::Vector3d fk_foot_base_ip1(data.fk_left_foot_pos_x[i+1],
                                         data.fk_left_foot_pos_y[i+1],
                                         data.fk_left_foot_pos_z[i+1]);
        
        // Base position in world frame
        Eigen::Vector3d base_pos_i(data.pos_x[i], data.pos_y[i], data.pos_z[i]);
        Eigen::Vector3d base_pos_ip1(data.pos_x[i+1], data.pos_y[i+1], data.pos_z[i+1]);
        
        // FK foot position in world frame
        Eigen::Vector3d fk_foot_world_i = base_pos_i + R_i * fk_foot_base_i;
        Eigen::Vector3d fk_foot_world_ip1 = base_pos_ip1 + R_ip1 * fk_foot_base_ip1;
        
        // Differentiate to get velocity
        double dt = data.timestamp[i+1] - data.timestamp[i];
        Eigen::Vector3d velocity = (fk_foot_world_ip1 - fk_foot_world_i) / dt;
        
        lfoot_velocity_FK.push_back(velocity);
        lfoot_velocity_FK_norm.push_back(velocity.norm() + 1e-8);
    }
    
    // Keep size consistent: duplicate the last velocity
    lfoot_velocity_FK.push_back(lfoot_velocity_FK.back());
    lfoot_velocity_FK_norm.push_back(lfoot_velocity_FK_norm.back());
}

// ---------------------------------------------------------------------------
// Contact region helpers
// ---------------------------------------------------------------------------
struct ContactRegion {
    size_t start;
    size_t end;   // inclusive
};

// Extract contiguous contact regions from a binary contact signal
std::vector<ContactRegion> extract_contact_regions(
    const std::vector<double>& contact_signal)
{
    std::vector<ContactRegion> regions;
    size_t N = contact_signal.size();
    size_t i = 0;
    while (i < N) {
        if (contact_signal[i] == 1) {
            size_t start = i;
            while (i < N && contact_signal[i] == 1) ++i;
            size_t end = i - 1; // inclusive
            regions.push_back({start, end});
        } else {
            ++i;
        }
    }
    return regions;
}

struct DetailedSample {
    double timestamp;
    Eigen::Vector3d foot_vel_world_GT;   // GT foot velocity from lfoot_pos
    Eigen::Vector3d foot_vel_world_FK;   // FK foot velocity from base_pos + R * fk_foot_pos
    Eigen::Vector3d base_vel_GT;         // Ground truth base velocity
    Eigen::Vector3d base_vel_FK;         // Naive FK estimate: -foot_vel_world_FK
    Eigen::Vector3d base_vel_kin;        // Kinematic estimate: Jacobian foot-fixed constraint
    Eigen::Vector3d base_vel_dyn;        // Dynamics estimate: integrated from KKT
    Eigen::Vector3d error_FK;            // base_vel_FK  - base_vel_GT
    Eigen::Vector3d error_kin;           // base_vel_kin - base_vel_GT
    Eigen::Vector3d error_dyn;           // base_vel_dyn - base_vel_GT
    double slip_indicator;               // ||base_vel_kin - base_vel_dyn||
    Eigen::Vector3d base_acc_kin;        // FD of kinematic velocity: (v_kin[i+1]-v_kin[i])/dt
    Eigen::Vector3d base_acc_dyn;        // KKT output (world frame) used for this step's integration
    size_t region_id;
};

// Collect detailed samples from contact region
void analyse_region(
    const DataStruct& data,
    const ContactRegion& region,
    const std::vector<Eigen::Vector3d>& lfoot_velocity_world_GT,
    const std::vector<Eigen::Vector3d>& lfoot_velocity_world_FK,
    RobotModel& robot,
    pinocchio::FrameIndex lfoot_frame_id,
    std::vector<DetailedSample>* detailed_samples,
    size_t region_id)
{
    const size_t len = region.end - region.start + 1;

    // -----------------------------------------------------------------------
    // Pass 1: pre-compute all kinematic velocities for this region
    // -----------------------------------------------------------------------
    std::vector<Eigen::Vector3d> v_kin_arr(len);
    for (size_t k = 0; k < len; ++k) {
        size_t i = region.start + k;
        Eigen::VectorXd q = build_configuration(data, i);
        v_kin_arr[k] = compute_base_velocity_kinematics(robot, q, data, i, lfoot_frame_id);
    }

    // Seed dynamics from kinematic estimate at region start
    Eigen::Vector3d v_b_dyn = v_kin_arr[0];

    std::cerr << "\n[Region " << region_id << "] t=[" << data.timestamp[region.start]
              << ", " << data.timestamp[region.end]
              << "] len=" << len << "\n"
              << "  seed v_b_dyn (world): " << v_b_dyn.transpose() << "\n"
              << "  GT v_base at start:   "
              << Eigen::Vector3d(data.vel_x[region.start],
                                 data.vel_y[region.start],
                                 data.vel_z[region.start]).transpose() << "\n";

    // -----------------------------------------------------------------------
    // Pass 2: compute dynamics accelerations, integrate, build samples
    // -----------------------------------------------------------------------
    for (size_t k = 0; k < len; ++k) {
        size_t i = region.start + k;

        Eigen::Vector3d v_base_gt(data.vel_x[i], data.vel_y[i], data.vel_z[i]);
        Eigen::Vector3d v_foot_world_FK = lfoot_velocity_world_FK[i];
        Eigen::Vector3d v_b_kin = v_kin_arr[k];

        Eigen::VectorXd q          = build_configuration(data, i);
        Eigen::VectorXd tau_joints = build_joint_torques(data, i);

        // Kinematic acceleration: forward FD of v_kin; backward FD at last sample
        Eigen::Vector3d a_kin = Eigen::Vector3d::Zero();
        if (k + 1 < len) {
            double dt = data.timestamp[i + 1] - data.timestamp[i];
            if (dt > 1e-9) a_kin = (v_kin_arr[k + 1] - v_kin_arr[k]) / dt;
        } else if (k > 0) {
            double dt = data.timestamp[i] - data.timestamp[i - 1];
            if (dt > 1e-9) a_kin = (v_kin_arr[k] - v_kin_arr[k - 1]) / dt;
        }

        // Dynamics acceleration from KKT (world frame), using current v_b_dyn
        Eigen::Vector3d a_dyn = compute_base_acceleration_dynamics(
            robot, q, data, i, tau_joints, v_b_dyn, lfoot_frame_id,
            /*debug=*/(k == 0));

        // Build and store sample (v_b_dyn is BEFORE integration — velocity at this step)
        DetailedSample sample;
        sample.timestamp         = data.timestamp[i];
        sample.foot_vel_world_GT = lfoot_velocity_world_GT[i];
        sample.foot_vel_world_FK = v_foot_world_FK;
        sample.base_vel_GT       = v_base_gt;
        sample.base_vel_FK       = -v_foot_world_FK;
        sample.base_vel_kin      = v_b_kin;
        sample.base_vel_dyn      = v_b_dyn;
        sample.error_FK          = -v_foot_world_FK - v_base_gt;
        sample.error_kin         = v_b_kin          - v_base_gt;
        sample.error_dyn         = v_b_dyn          - v_base_gt;
        // One-step prediction residual: does dynamics predict the next kinematic velocity?
        // s(k) = ||v_kin[k+1] - (v_kin[k] + a_dyn[k] * dt)||
        sample.slip_indicator = 0.0;
        if (k + 1 < len) {
            double dt = data.timestamp[i + 1] - data.timestamp[i];
            Eigen::Vector3d v_kin_next_pred = v_b_kin + a_dyn * dt;
            sample.slip_indicator = (v_kin_arr[k + 1] - v_kin_next_pred).norm();
        }
        sample.base_acc_kin      = a_kin;
        sample.base_acc_dyn      = a_dyn;
        sample.region_id         = region_id;
        detailed_samples->push_back(sample);

        // Euler integration to get v_b_dyn for next step
        if (k + 1 < len) {
            double dt = data.timestamp[i + 1] - data.timestamp[i];
            v_b_dyn += a_dyn * dt;
        }
    }
}

int main(int argc, char** argv) {
    std::cout << "=== Dynamic Kinematic Slip Analysis ===\n" << std::endl;

    // Load CSV file
    std::string csv_path = (argc > 1) ? argv[1] : "../Data/robotstate_0_800.csv";
    DataStruct data = ParseCSV(csv_path);
    std::cout << "Loaded " << data.num_rows << " rows from: " << csv_path << "\n" << std::endl;

    // Load robot model (used for name/config and Pinocchio model)
    std::string config_path = "../config/G1_config.yaml";
    RobotModel robot;
    if (!robot.load_config(config_path)) {
        std::cerr << "Error: Failed to load robot configuration!" << std::endl;
        return -1;
    }
    std::cout << "Robot model: " << robot.robot_name << std::endl;
    std::cout << "Pinocchio model: " << robot.pin_model.name << std::endl;
    std::cout << "Total DOF: " << robot.pin_model.nv << std::endl;
    
    // Get left foot frame ID (first contact point is left foot)
    pinocchio::FrameIndex lfoot_frame_id = robot.foot_frame_ids[0];
    std::cout << "Left foot frame ID: " << lfoot_frame_id << "\n" << std::endl;

    // -------------------------------------------------------------------------
    // Compute left foot velocities
    // -------------------------------------------------------------------------
    std::vector<Eigen::Vector3d> lfoot_velocity_world_GT, lfoot_velocity_world_FK;
    std::vector<double> lfoot_velocity_norm_GT, lfoot_velocity_norm_FK;
    
    compute_lfoot_velocity_world_GT(data, lfoot_velocity_world_GT, lfoot_velocity_norm_GT);
    compute_lfoot_velocity_world_FK(data, lfoot_velocity_world_FK, lfoot_velocity_norm_FK);
    
    std::cout << "Computed GT foot velocity from lfoot_pos" << std::endl;
    std::cout << "Computed FK foot velocity from base_pos + R * fk_foot_pos" << std::endl;

    // -------------------------------------------------------------------------
    // 1. Extract left foot contact regions from ground truth contact signal
    // -------------------------------------------------------------------------
    auto lfoot_regions = extract_contact_regions(data.lfoot_contact);

    std::cout << "Left foot contact regions: " << lfoot_regions.size() << "\n" << std::endl;

    // -------------------------------------------------------------------------
    // 2. Analyse each contact region
    // -------------------------------------------------------------------------
    std::vector<DetailedSample> detailed_samples;
    size_t region_idx = 0;

    for (const auto& region : lfoot_regions) {
        analyse_region(data, region, lfoot_velocity_world_GT, lfoot_velocity_world_FK,
                       robot, lfoot_frame_id, &detailed_samples, ++region_idx);
    }

    // -------------------------------------------------------------------------
    // 3. Write detailed data to CSV for plotting
    // -------------------------------------------------------------------------
    // std::string output_csv = "../Extra_Files/DynKinSlipAnalysis/contact_analysis_output.csv";
    std::string output_csv = "contact_analysis_output.csv";
    
    std::ofstream outfile(output_csv);
    if (outfile.is_open()) {
        outfile << "timestamp,region_id,"
                << "foot_vel_GT_x,foot_vel_GT_y,foot_vel_GT_z,"
                << "foot_vel_FK_x,foot_vel_FK_y,foot_vel_FK_z,"
                << "base_vel_GT_x,base_vel_GT_y,base_vel_GT_z,"
                << "base_vel_FK_x,base_vel_FK_y,base_vel_FK_z,"
                << "base_vel_kin_x,base_vel_kin_y,base_vel_kin_z,"
                << "base_vel_dyn_x,base_vel_dyn_y,base_vel_dyn_z,"
                << "error_FK_x,error_FK_y,error_FK_z,"
                << "error_kin_x,error_kin_y,error_kin_z,"
                << "error_dyn_x,error_dyn_y,error_dyn_z,"
                << "slip_indicator,"
                << "base_acc_kin_x,base_acc_kin_y,base_acc_kin_z,"
                << "base_acc_dyn_x,base_acc_dyn_y,base_acc_dyn_z\n";
        
        for (const auto& sample : detailed_samples) {
            outfile << std::fixed << std::setprecision(6)
                    << sample.timestamp << "," << sample.region_id << ","
                    << sample.foot_vel_world_GT.x() << ","
                    << sample.foot_vel_world_GT.y() << ","
                    << sample.foot_vel_world_GT.z() << ","
                    << sample.foot_vel_world_FK.x() << ","
                    << sample.foot_vel_world_FK.y() << ","
                    << sample.foot_vel_world_FK.z() << ","
                    << sample.base_vel_GT.x() << ","
                    << sample.base_vel_GT.y() << ","
                    << sample.base_vel_GT.z() << ","
                    << sample.base_vel_FK.x() << ","
                    << sample.base_vel_FK.y() << ","
                    << sample.base_vel_FK.z() << ","
                    << sample.base_vel_kin.x() << ","
                    << sample.base_vel_kin.y() << ","
                    << sample.base_vel_kin.z() << ","
                    << sample.base_vel_dyn.x() << ","
                    << sample.base_vel_dyn.y() << ","
                    << sample.base_vel_dyn.z() << ","
                    << sample.error_FK.x() << ","
                    << sample.error_FK.y() << ","
                    << sample.error_FK.z() << ","
                    << sample.error_kin.x() << ","
                    << sample.error_kin.y() << ","
                    << sample.error_kin.z() << ","
                    << sample.error_dyn.x() << ","
                    << sample.error_dyn.y() << ","
                    << sample.error_dyn.z() << ","
                    << sample.slip_indicator << ","
                    << sample.base_acc_kin.x() << ","
                    << sample.base_acc_kin.y() << ","
                    << sample.base_acc_kin.z() << ","
                    << sample.base_acc_dyn.x() << ","
                    << sample.base_acc_dyn.y() << ","
                    << sample.base_acc_dyn.z() << "\n";
        }
        outfile.close();
        std::cout << "\nDetailed analysis written to: " << output_csv << std::endl;
        std::cout << "Total samples: " << detailed_samples.size() << std::endl;
        
        // -------------------------------------------------------------------------
        // 4. Compute summary statistics
        // -------------------------------------------------------------------------
        double mean_error_FK  = 0.0, mean_error_kin  = 0.0, mean_error_dyn  = 0.0;
        double max_error_FK   = 0.0, max_error_kin   = 0.0, max_error_dyn   = 0.0;
        double mean_slip      = 0.0, max_slip         = 0.0;
        
        for (const auto& sample : detailed_samples) {
            double err_FK  = sample.error_FK.norm();
            double err_kin = sample.error_kin.norm();
            double err_dyn = sample.error_dyn.norm();
            mean_error_FK  += err_FK;
            mean_error_kin += err_kin;
            mean_error_dyn += err_dyn;
            max_error_FK   = std::max(max_error_FK,  err_FK);
            max_error_kin  = std::max(max_error_kin, err_kin);
            max_error_dyn  = std::max(max_error_dyn, err_dyn);
            mean_slip      += sample.slip_indicator;
            max_slip        = std::max(max_slip, sample.slip_indicator);
        }
        mean_error_FK  /= detailed_samples.size();
        mean_error_kin /= detailed_samples.size();
        mean_error_dyn /= detailed_samples.size();
        mean_slip      /= detailed_samples.size();
        
        std::cout << "\n=== Summary Statistics ===" << std::endl;
        std::cout << "FK-based estimates (naive, -v_foot):" << std::endl;
        std::cout << "  Mean error: " << mean_error_FK  << " m/s" << std::endl;
        std::cout << "  Max error:  " << max_error_FK   << " m/s" << std::endl;
        std::cout << "\nKinematic estimates (Jacobian, foot-fixed constraint):" << std::endl;
        std::cout << "  Mean error: " << mean_error_kin << " m/s" << std::endl;
        std::cout << "  Max error:  " << max_error_kin  << " m/s" << std::endl;
        std::cout << "\nDynamics estimates (KKT integration, torque-driven):" << std::endl;
        std::cout << "  Mean error: " << mean_error_dyn << " m/s" << std::endl;
        std::cout << "  Max error:  " << max_error_dyn  << " m/s" << std::endl;
        std::cout << "\nSlip indicator ||v_kin - v_dyn||:" << std::endl;
        std::cout << "  Mean: " << mean_slip << " m/s" << std::endl;
        std::cout << "  Max:  " << max_slip  << " m/s" << std::endl;
    } else {
        std::cerr << "Error: Could not write output CSV file" << std::endl;
    }

    return 0;
}
 


