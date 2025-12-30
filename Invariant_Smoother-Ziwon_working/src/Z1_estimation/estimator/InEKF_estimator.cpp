/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   kinematics.cpp
 *  @author Ross Hartley
 *  @brief  Example of invariant filtering for contact-aided inertial navigation
 *  @date   September 25, 2018 my birthday
 **/

#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <boost/algorithm/string.hpp>
#include <vector>
#include "InEKF_estimator.hpp"

#define DT_MIN 1e-6
#define DT_MAX 1

using std::cout;  
using std::endl;
//using namespace inekf;




// This is just reading of the sensor data file and ground truth data file

void InEKFilter::LoadCSVData(const std::string& csv_filename) {
    csvData = ParseDigitCSV(csv_filename);
    std::cout << "Loaded CSV data with " << csvData.timestamp.size() << " rows." << std::endl;
    textfile_flag  = true;
}




// Assign covariances for contact points and see if they exceed slip threshold 
// or keep it variable based on residual velocity between encoders and IMU
// Ask why foot covariance is self confirming!!
// Have a good conversation with Ziwon about this

std::vector<double> InEKFilter::Variable_Contact_Cov(int time){ // time is either 0 or 1 cause sliding window of 2

    std::vector<double> contact_cov_array;
    contact_cov_array.clear(); // Redundant if you ask me

    for(int k=0; k<robot.leg_no; k++){

        double contact_cov = robot.cov_contact_const;

        //  only legs with actual contact get potentially adjusted covariance.
        //  Swing state legs get assingined a default value for consistent index purposes
        if(HARD_CONTACT_t[time](k)){ 

            if(variable_contact_cov_mode){

                contact_cov = d_v[time].block(3*k,0,3,1).norm();
                //Check below for how d_v is computed

                // Contact covariance increases exponentially with residual velocity which ideally should be zero
                // 10^( log10(cov_contact_const) + ||d_v|| * cov_amplifier )
                contact_cov = std::pow(10, log10(robot.cov_contact_const) + contact_cov*cov_amplifier );

                if( contact_cov > robot.cov_slip_const ){
                    contact_cov = robot.cov_slip_const;
                }

            }else if(slip_rejection_mode){

                // If the residual velocity exceeds the slip threshold, assign a high covariance
                if( (d_v[time].block(3*k,0, 3,1).norm() > slip_threshold)){
                    contact_cov = robot.cov_slip_const;
                }else{
                    // otherwise assign the default contact covariance
                    contact_cov = robot.cov_contact_const;
                }

            }else{
                // if neither mode is on, just assign the default contact covariance
                contact_cov = robot.cov_contact_const;
            }

        }

        //push back is like append for each iteration of the leg
        contact_cov_array.push_back( contact_cov );

    }

    return contact_cov_array;
}





void InEKFilter::Initialize(double _dt, Eigen::Matrix<double,12,1> cov_val_setting, Eigen::Matrix<double, 16, 1> initial_condition)
{

    //  ---- Initialize invariant extended Kalman filter ----- //

    filter.Reset();

    sliding_window_flag = false;
    frame_count = 0;
    time_count = 0;
    robot.Covariance_Reset(cov_val_setting); // Set covariance of the 12x1 vector input
    robot.dt = _dt;

    dt = robot.dt;
    gravity << 0,0, -9.80665;


        // // hardcoded is 1e-3, but initialized in main cpp is 10^-10....
        // std::cout << "robot.cov_prior_orientation_const: " << robot.cov_prior_orientation_const << std::endl;


    std::string mode;
    if(variable_contact_cov_mode == true){
        mode="(CS" + BasicFunctions_Estimator::to_string_n_signficant_figures(cov_amplifier,2)+ ")";
    }else if(slip_rejection_mode == true){
        mode="(SR" + BasicFunctions_Estimator::to_string_n_signficant_figures(slip_threshold,2) + ")";
    }
    // print mode info
    // e.g. InEKF(SR0.5)H(adj)(0_0_0_0_0_0_0_0_0_0_0_0)

    estimator_info = mode+"H(adj)";
    initial_info = "(" + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[0], 2) + "_" + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[1], 2) + "_"
                        + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[2], 2) + "_" + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[3], 2) + "_"
                        + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[4], 2) + "_" + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[5], 2) + "_"
                        + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[6], 2) + "_"+ BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[7], 2) + "_"
                        + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[8], 2) + "_"+ BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[9], 2) + "_"
                        + BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[10], 2) + "_"+ BasicFunctions_Estimator::to_string_n_figures_under_0(initial_condition[11], 2) + ")";


    //covariance setting, everything is diagonal and constant for each element
    // 15x15 covariance matrix for the state [R, v, p, bg, ba]
    Eigen::Matrix<double,15,15> ForP; 
    ForP.setZero();
    ForP.block(0,0,9,9) = robot.cov_prior_orientation_const * Eigen::MatrixXd::Identity(9,9); // orientation, position, velocity all have same prior covariance
    ForP.block(9,9,6,6) = robot.cov_prior_bias_gyro_const * Eigen::MatrixXd::Identity(6,6); // bias gyro, bias accel all have same prior covariance


    Eigen::Matrix3d R0;
    Eigen::Vector3d v0, p0, bg0, ba0;

    inekf::RobotState initial_state(ForP);

    if(textfile_flag){

        // initialize from CSV data + initial condition offset
        int idx = 0 + gt_sd; // changed to 0 + gt_sd from 1 + gt_sd
        
        // Position from CSV data (pos_x, pos_y, pos_z)
        p0 << csvData.pos_x[idx] + initial_condition(0),
              csvData.pos_y[idx] + initial_condition(1),
              csvData.pos_z[idx] + initial_condition(2);
        
        // Velocity from CSV data (vel_x, vel_y, vel_z)
        v0 << csvData.vel_x[idx] + initial_condition(7),
              csvData.vel_y[idx] + initial_condition(8),
              csvData.vel_z[idx] + initial_condition(9);

        // Bias initialization
        bg0 << initial_condition(10), initial_condition(11), initial_condition(12);
        ba0 << initial_condition(13), initial_condition(14), initial_condition(15);

        // Convert quaternion from CSV to rotation matrix
        Eigen::Vector4d quat;
        quat << csvData.quat_w[idx], csvData.quat_i[idx], csvData.quat_j[idx], csvData.quat_k[idx];
        R0 = BasicFunctions_Estimator::Quaternion_to_Rotation_Matrix(quat);

        Vector3d Euler = BasicFunctions_Estimator::Rotation_to_EulerZYX(R0);
        // add initial condition offset
        Euler(0) = Euler(0) + initial_condition[3];
        Euler(1) = Euler(1) + initial_condition[4];
        Euler(2) = Euler(2) + initial_condition[5];
        R0 = BasicFunctions_Estimator::EulerZYX_to_R_bw(Euler);
        // std::cout << "R0 (Rotation Matrix) from CSV: \n" << R0 << std::endl;

    }else{

        Eigen::Vector4d initial_quaternion = initial_condition.block(3,0,4,1);
        R0=BasicFunctions_Estimator::Quaternion_to_Rotation_Matrix(initial_quaternion);
        v0.setZero(); // initial velocity
        p0=initial_condition.block(0,0,3,1);
        bg0.setZero(); // initial gyroscope bias
        ba0.setZero(); // initial accelerometer bias

    }


    initial_state.setRotation(R0);
    initial_state.setVelocity(v0);
    initial_state.setPosition(p0);
    initial_state.setGyroscopeBias(bg0);
    initial_state.setAccelerometerBias(ba0);


    imu_measurement = Eigen::Matrix<double,6,1>::Zero();
    imu_measurement_prev = Eigen::Matrix<double,6,1>::Zero(); // its 

    // Initialize state covariance
    inekf::NoiseParams noise_params;
    noise_params.setGyroscopeNoise(sqrt(robot.cov_gyro_const));
    noise_params.setAccelerometerNoise(sqrt(robot.cov_acc_const));
    noise_params.setGyroscopeBiasNoise(sqrt(robot.cov_bias_gyro_const));
    noise_params.setAccelerometerBiasNoise(sqrt(robot.cov_bias_acc_const));
    noise_params.setContactNoise(sqrt(robot.cov_contact_const));
    noise_params.setLandmarkNoise(sqrt(robot.cov_contact_const));

    // Initialize filter
    filter.setState(initial_state);
    filter.setNoiseParams(noise_params);

   cout << "Noise parameters are initialized to: \n";
   cout << filter.getNoiseParams() << endl;
   cout << "Robot's state is initialized to: \n";
   cout << filter.getState() << endl;
   // Theta is bias of sensors 6x1 vector

//    cout << R0 << endl;
//    cout << "Initial position (x,y,z): " << p0.transpose() << endl;
//    cout << "Initial velocity (x,y,z): " << v0.transpose() << endl;

}



// Inputs are 30x1 sensor measurement vector and 2x1 contact state vector
void InEKFilter::new_measurement(Eigen::Matrix<double,30,1> &Sensor_i, Eigen::Matrix<bool,2,1> &Contact_i)
{


    //sensor measurement reading
    if(textfile_flag)
    {

        // int j = time_count + 1; // figure out why +1 leave like this for now

        int j = time_count;


        // IMU data from CSV (gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z)
        IMU_Gyro[frame_count] << csvData.gyro_x[j], csvData.gyro_y[j], csvData.gyro_z[j];
        IMU_Acc[frame_count] << csvData.acc_x[j], csvData.acc_y[j], csvData.acc_z[j];





        // Populate ENCODER with 61 configuration values for Pinocchio FK
        // q[0:7] → baseJoint_0 (position + quaternion) - set to identity (0,0,0,0,0,0,1)
        ENCODER[frame_count](0) = 0.0;
        ENCODER[frame_count](1) = 0.0;
        ENCODER[frame_count](2) = 0.0;
        ENCODER[frame_count](3) = 0.0;
        ENCODER[frame_count](4) = 0.0;
        ENCODER[frame_count](5) = 0.0;
        ENCODER[frame_count](6) = 1.0;

        // Augment base position and rotation to Encoder for FK (Create a variable for just digit encoders and then populate larger vector internally)
        
        // q[7:10] → Left hip (roll, yaw, pitch)
        ENCODER[frame_count](7) = csvData.left_hip_roll[j];
        ENCODER[frame_count](8) = csvData.left_hip_yaw[j];
        ENCODER[frame_count](9) = csvData.left_hip_pitch[j];
        
        // q[10:14] → left-achilles-rod (spherical joint) - set to zeros
        ENCODER[frame_count](10) = 0.0;
        ENCODER[frame_count](11) = 0.0;
        ENCODER[frame_count](12) = 0.0;
        ENCODER[frame_count](13) = 0.0;
        
        // q[14:18] → Left knee, shin, tarsus, heel-spring
        ENCODER[frame_count](14) = csvData.left_knee[j];
        ENCODER[frame_count](15) = csvData.left_shin[j];
        ENCODER[frame_count](16) = csvData.left_tarsus[j];
        ENCODER[frame_count](17) = csvData.left_heel_spring[j];
        
        // q[18:19] → left-toe-A
        ENCODER[frame_count](18) = csvData.left_toe_A[j];
        
        // q[19:23] → left-toe-A-rod (spherical joint) - set to zeros
        ENCODER[frame_count](19) = 0.0;
        ENCODER[frame_count](20) = 0.0;
        ENCODER[frame_count](21) = 0.0;
        ENCODER[frame_count](22) = 0.0;
        
        // q[23] → left-toe-B
        ENCODER[frame_count](23) = csvData.left_toe_B[j];
        
        // q[24:28] → left-toe-B-rod (spherical joint) - set to zeros
        ENCODER[frame_count](24) = 0.0;
        ENCODER[frame_count](25) = 0.0;
        ENCODER[frame_count](26) = 0.0;
        ENCODER[frame_count](27) = 0.0;
        
        // q[28:30] → left toe pitch and roll
        ENCODER[frame_count](28) = csvData.left_toe_pitch[j];
        ENCODER[frame_count](29) = csvData.left_toe_roll[j];
        
        // q[30:34] → Left arm (shoulder roll, pitch, yaw, elbow) - set to zeros for now
        ENCODER[frame_count](30) = 0.0; // csvData.left_shoulder_roll[j];
        ENCODER[frame_count](31) = 0.0; // csvData.left_shoulder_pitch[j];
        ENCODER[frame_count](32) = 0.0; // csvData.left_shoulder_yaw[j];
        ENCODER[frame_count](33) = 0.0; // csvData.left_elbow[j];
        
        // q[34:37] → Right hip (roll, yaw, pitch)
        ENCODER[frame_count](34) = csvData.right_hip_roll[j];
        ENCODER[frame_count](35) = csvData.right_hip_yaw[j];
        ENCODER[frame_count](36) = csvData.right_hip_pitch[j];
        
        // q[37:41] → right-achilles-rod (spherical joint) - set to zeros
        ENCODER[frame_count](37) = 0.0;
        ENCODER[frame_count](38) = 0.0;
        ENCODER[frame_count](39) = 0.0;
        ENCODER[frame_count](40) = 0.0;
        
        // q[41:45] → Right knee, shin, tarsus, heel-spring
        ENCODER[frame_count](41) = csvData.right_knee[j];
        ENCODER[frame_count](42) = csvData.right_shin[j];
        ENCODER[frame_count](43) = csvData.right_tarsus[j];
        ENCODER[frame_count](44) = csvData.right_heel_spring[j];
        
        // q[45] → right-toe-A
        ENCODER[frame_count](45) = csvData.right_toe_A[j];
        
        // q[46:50] → right-toe-A-rod (spherical joint) - set to zeros
        ENCODER[frame_count](46) = 0.0;
        ENCODER[frame_count](47) = 0.0;
        ENCODER[frame_count](48) = 0.0;
        ENCODER[frame_count](49) = 0.0;
        
        // q[50] → right-toe-B
        ENCODER[frame_count](50) = csvData.right_toe_B[j];
        
        // q[51:55] → right-toe-B-rod (spherical joint) - set to zeros
        ENCODER[frame_count](51) = 0.0;
        ENCODER[frame_count](52) = 0.0;
        ENCODER[frame_count](53) = 0.0;
        ENCODER[frame_count](54) = 0.0;
        
        // q[55:57] → right toe pitch and roll
        ENCODER[frame_count](55) = csvData.right_toe_pitch[j];
        ENCODER[frame_count](56) = csvData.right_toe_roll[j];
        
        // q[57:61] → Right arm (shoulder roll, pitch, yaw, elbow) - set to zeros for now
        ENCODER[frame_count](57) = 0.0; // csvData.right_shoulder_roll[j];
        ENCODER[frame_count](58) = 0.0; // csvData.right_shoulder_pitch[j];
        ENCODER[frame_count](59) = 0.0; // csvData.right_shoulder_yaw[j];
        ENCODER[frame_count](60) = 0.0; // csvData.right_elbow[j];

        // ENCODERDOT - populate with velocity data (55 velocity DOFs)
        // For now, using available joint velocities from CSV
        // Base velocities (linear) - set to zero (base should not move)
        ENCODERDOT[frame_count](0) = 0.0;
        ENCODERDOT[frame_count](1) = 0.0;
        ENCODERDOT[frame_count](2) = 0.0;
        // Base angular velocities - set to zero (base should not move)
        ENCODERDOT[frame_count](3) = 0.0;
        ENCODERDOT[frame_count](4) = 0.0;
        ENCODERDOT[frame_count](5) = 0.0;
        
        // Left leg joint velocities
        ENCODERDOT[frame_count](6) = csvData.left_hip_roll_vel[j];
        ENCODERDOT[frame_count](7) = csvData.left_hip_yaw_vel[j];
        ENCODERDOT[frame_count](8) = csvData.left_hip_pitch_vel[j];
        ENCODERDOT[frame_count](9) = 0.0; // achilles-rod
        ENCODERDOT[frame_count](10) = csvData.left_knee_vel[j];
        ENCODERDOT[frame_count](11) = csvData.left_shin_vel[j];
        ENCODERDOT[frame_count](12) = csvData.left_tarsus_vel[j];
        ENCODERDOT[frame_count](13) = csvData.left_heel_spring_vel[j];
        ENCODERDOT[frame_count](14) = csvData.left_toe_A_vel[j];
        ENCODERDOT[frame_count](15) = 0.0; // toe-A-rod
        ENCODERDOT[frame_count](16) = csvData.left_toe_B_vel[j];
        ENCODERDOT[frame_count](17) = 0.0; // toe-B-rod
        ENCODERDOT[frame_count](18) = csvData.left_toe_pitch_vel[j];
        ENCODERDOT[frame_count](19) = csvData.left_toe_roll_vel[j];
        
        // Left arm velocities - set to zeros
        ENCODERDOT[frame_count](20) = 0.0;
        ENCODERDOT[frame_count](21) = 0.0;
        ENCODERDOT[frame_count](22) = 0.0;
        ENCODERDOT[frame_count](23) = 0.0;
        
        // Right leg joint velocities
        ENCODERDOT[frame_count](24) = csvData.right_hip_roll_vel[j];
        ENCODERDOT[frame_count](25) = csvData.right_hip_yaw_vel[j];
        ENCODERDOT[frame_count](26) = csvData.right_hip_pitch_vel[j];
        ENCODERDOT[frame_count](27) = 0.0; // achilles-rod
        ENCODERDOT[frame_count](28) = csvData.right_knee_vel[j];
        ENCODERDOT[frame_count](29) = csvData.right_shin_vel[j];
        ENCODERDOT[frame_count](30) = csvData.right_tarsus_vel[j];
        ENCODERDOT[frame_count](31) = csvData.right_heel_spring_vel[j];
        ENCODERDOT[frame_count](32) = csvData.right_toe_A_vel[j];
        ENCODERDOT[frame_count](33) = 0.0; // toe-A-rod
        ENCODERDOT[frame_count](34) = csvData.right_toe_B_vel[j];
        ENCODERDOT[frame_count](35) = 0.0; // toe-B-rod
        ENCODERDOT[frame_count](36) = csvData.right_toe_pitch_vel[j];
        ENCODERDOT[frame_count](37) = csvData.right_toe_roll_vel[j];
        
        // Right arm velocities - set to zeros
        ENCODERDOT[frame_count](38) = 0.0;
        ENCODERDOT[frame_count](39) = 0.0;
        ENCODERDOT[frame_count](40) = 0.0;
        ENCODERDOT[frame_count](41) = 0.0;

        // Contact states for 2 legs (left and right foot)
        CONTACT_t[frame_count] << csvData.lfoot_contact[j], csvData.rfoot_contact[j];

        imu_measurement << IMU_Gyro[frame_count], IMU_Acc[frame_count];

    }else{

        IMU_Gyro[frame_count] = Sensor_i.block(0,0,3,1);
        IMU_Acc[frame_count] = Sensor_i.block(3,0,3,1);

        ENCODER[frame_count] = Sensor_i.block(6,0,12,1);
        //ENCODER[frame_count] << Sensor_i.block(6,0,3,1),Sensor_i.block(6,0,3,1),Sensor_i.block(6,0,3,1),Sensor_i.block(6,0,3,1);

        ENCODERDOT[frame_count] =Sensor_i.block(18,0,12,1);
        CONTACT_t[frame_count] = Contact_i;

        imu_measurement <<IMU_Gyro[frame_count], IMU_Acc[frame_count];

    }

    //compute forward kinematics and joint Jacobians as soon as you get new measurement
    robot.Forward_Kinematics(ENCODER[frame_count], ENCODERDOT[frame_count]);
    robot.compute_joint_jacobians();

    //propagate-----------------------------------------------------------------------------

    //Slip Rejection-------------------------------------------------------------------------------

    if ( frame_count==0 ){

        for (int k=0; k<robot.leg_no; k++){
            SLIP_t[0](k) = false;
            HARD_CONTACT_t[0](k)  = CONTACT_t[0](k)-SLIP_t[0](k); // Actual hard contacts are contact - slip (what is this logic?)(true - false = true, true - true = false)
        }

    }else{

        Eigen::Vector3d phi = (imu_measurement.block(0,0, 3,1)-filter.getState().getGyroscopeBias())*dt; // (angular increment - gyro bias)(*dt)
        Rotation_s[1] = filter.getState().getRotation() * BasicFunctions_Estimator::Expm_Vec(phi); // Rotation update with gyro bias correction
        JacobiSVD<Matrix3d> svd(Rotation_s[frame_count], ComputeFullU|ComputeFullV); // Cause we're doing dt, we need to re-orthogonalize the rotation matrix back to its manifold
        Rotation_s[1] = svd.matrixU() * svd.matrixV().transpose();

        Velocity_s[1] = filter.getState().getVelocity() + (Rotation_s[1]*(imu_measurement.block(3,0, 3,1)-filter.getState().getAccelerometerBias()) + gravity)*dt;
        // Velocity update with accel bias correction and gravity


//        double contact_dv_mean = 0;
//        int count = 0;
//
//        for (int k=0; k<4; k++){
//
//            d_v[1].block(3*k,0,3,1) = Velocity_s[1]
//                    + Rotation_s[1]*robot.Jacobian_Leg(ENCODER[1].block<3,1>(3*k,0), k+1)*ENCODERDOT[1].block<3,1>(3*k,0)
//                    + Rotation_s[1]*BasicFunctions_Estimator::Hat_so3(IMU_Gyro[1] - Bias_Gyro_s[1])*robot.Forward_Kinematics_Leg(ENCODER[1].block<3,1>(3*k,0), k+1);
//
//            HARD_CONTACT_t[1](k)  = CONTACT_t[1](k);
//
//            if ( slip_rejection_mode == true && CONTACT_t[1](k) == true &&
//                 !SLIP_t[1](k) ){
//                contact_dv_mean += (d_v[0].block(3*k,0, 3,1).norm() - contact_dv_mean)/(count+1);
//                count++;
//            }
//
////            HARD_CONTACT_t[1](k)  = CONTACT_t[1](k)-SLIP_t[1](k);
//        }
//        //robot.slip_threshold = contact_dv_mean*2;
        // std::cout << "Frame count in new measurement is 1" << std::endl;
        // // Compute forward kinematics and joint Jacobians
        // robot.Forward_Kinematics(ENCODER[1], ENCODERDOT[1]);
        // robot.compute_joint_jacobians();

        // why am i computing dv only when frame_count == 1? cause we do have dv at frame_count == 0 right after initialization?;
        





    }

        for (int k=0; k<robot.leg_no; k++) {

        d_v[1].block(3*k,0,3,1) = Velocity_s[1]
                                    + Rotation_s[1]*robot.get_foot_velocity(k+1)
                                    + Rotation_s[1]*BasicFunctions_Estimator::Hat_so3(IMU_Gyro[1] - Bias_Gyro_s[1])*robot.get_foot_position(k+1);

        HARD_CONTACT_t[1](k)  = CONTACT_t[1](k);

        if ((slip_rejection_mode == true) && (CONTACT_t[1](k) == true) &&
            (d_v[1].block(3 * k, 0, 3, 1).norm() > robot.slip_threshold)) {
            SLIP_t[1](k) = true;
        }

//            HARD_CONTACT_t[1](k)  = CONTACT_t[1](k)-SLIP_t[1](k);
        }




}



void InEKFilter::Propagate_Correct()
{

    if(frame_count>0) {

        filter.Propagate(imu_measurement_prev, dt, Variable_Contact_Cov(0)); // propagate with previous imu measurement and dt

    }

    // change imu measurement_prev after inputting to propagation
    imu_measurement_prev = imu_measurement;


    // Rewrite contact list for filter
        std::
    vector<pair<int,bool> > contacts;

        for (int i=0; i<robot.leg_no; i++) {
            int id = i;
            bool indicator = HARD_CONTACT_t[frame_count](i);

            contacts.push_back(pair<int,bool> (id, indicator));
        }

    //cout<<"HC "<<HARD_CONTACT_t[frame_count]<<endl;
    filter.setContacts(contacts);


    int id;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double,6,6> covariance;
    inekf::vectorKinematics measured_kinematics;
    Eigen::Matrix3d Covariance_Encoder_leg = robot.cov_enc_const * Eigen::Matrix3d::Identity();

    // std::cout << "Frame count in propagate_correct first place: " << frame_count << std::endl;
    


    // Build measured kinematics from batch results
    for (int k=0; k<robot.leg_no; k++) {
        id = k;
        pose.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
        pose.block<3,1>(0,3) = robot.get_foot_position(k+1);
        
        // Extract Jacobian for this leg and compute measurement covariance
        Eigen::MatrixXd FK_Jacobian_full = robot.get_foot_jacobian(k+1);
        // Take top 3 rows (position) and remove first 6 columns (base)
        Eigen::MatrixXd FK_Jacobian = FK_Jacobian_full.block(0, 6, 3, FK_Jacobian_full.cols() - 6);
        // check size and implementation later
        
        // Compute diagonal covariance from norm squared of Jacobian rows
        Eigen::Vector3d jac_norm_sq;
        for (int axis = 0; axis < 3; axis++) {
            jac_norm_sq(axis) = FK_Jacobian.row(axis).squaredNorm();
        }
        covariance.block(3,3, 3,3) = jac_norm_sq.asDiagonal() * robot.cov_enc_const;

        inekf::Kinematics frame(id, pose, covariance);
        measured_kinematics.push_back(frame);
    }
    // Correct state using kinematic measurements

    bool flag = true; // should be false for loop below to work properly
    // if(time_count%10==5){ // correct every 10th step from 5, 15, 25, ..etc
    //     flag = true;
    // }

    filter.CorrectKinematics(measured_kinematics, flag);


    // std::cout << "Frame count in propagate_correct end: " << frame_count << std::endl;

    // std::cout << "d_v[0]: \n" << d_v[0] << std::endl;
    // std::cout << "d_v[1]: \n" << d_v[1] << std::endl;


        // This is a big one below cause when frame_count == 0 in the beginning, how can we access ENCODER[1], ENCODERDOT[1], IMU_Gyro[1]?
        // Changed all 1 values to frame_count for indexing

        // for (int k=0; k<robot.leg_no; k++){
        // d_v[1].block(3*k,0,3,1) = filter.getState().getVelocity()
                // + filter.getState().getRotation()*robot.Jacobian_Leg(ENCODER[1].block<3,1>(3*k,0), k+1)*ENCODERDOT[1].block<3,1>(3*k,0)
                // + filter.getState().getRotation()*BasicFunctions_Estimator::Hat_so3(IMU_Gyro[1] - filter.getState().getGyroscopeBias())*robot.Forward_Kinematics_Leg(ENCODER[1].block<3,1>(3*k,0), k+1);
    
    if (frame_count != 0){ 

        for (int k=0; k<robot.leg_no; k++){
            
            d_v[frame_count].block(3*k,0,3,1) = filter.getState().getVelocity()
                    + filter.getState().getRotation()*robot.get_foot_velocity(k+1)
                    + filter.getState().getRotation()*BasicFunctions_Estimator::Hat_so3(IMU_Gyro[frame_count] - filter.getState().getGyroscopeBias())*robot.get_foot_position(k+1);


            //Test Junny
            SLIP_t[frame_count](k) = false;
            if (slip_rejection_mode == true && CONTACT_t[frame_count](k) == true &&
                d_v[frame_count].block(3*k,0, 3,1).norm() > slip_threshold){
                SLIP_t[frame_count](k) = true;
            }
            HARD_CONTACT_t[frame_count](k)  = CONTACT_t[frame_count](k);

        }
    }   





    //cout << filter.getState().getX() << endl;

}


void InEKFilter::send_states(ROBOT_STATES &state_){


    //Sending Estimated States----------------------------------------------------------------------------

    Eigen::Vector3d W_pos_imu2bd = filter.getState().getRotation() * robot.IMU2BD;

    state_.Position = filter.getState().getPosition() + W_pos_imu2bd;

    Eigen::Vector3d w_gyro;
    w_gyro = filter.getState().getRotation()*(IMU_Gyro[1] - filter.getState().getGyroscopeBias());


    state_.Velocity = filter.getState().getVelocity() + w_gyro.cross(W_pos_imu2bd);
    state_.Bias_Gyro = filter.getState().getGyroscopeBias();
    state_.Bias_Acc = filter.getState().getAccelerometerBias();
    state_.Rotation= filter.getState().getRotation();


    if(frame_count == 0)
    {
        state_.Hard_Contact = HARD_CONTACT_t[frame_count];
        state_.Contact = CONTACT_t[frame_count];
        state_.Slip = SLIP_t[frame_count];
        state_.d_v = d_v[frame_count];
    }
    else
    {
        state_.Hard_Contact = HARD_CONTACT_t[frame_count-1];
        state_.Contact = CONTACT_t[frame_count-1];
        state_.Slip = SLIP_t[frame_count-1];
        state_.d_v = d_v[frame_count-1];
    }

    // std::cout << "Estimated Gyro Bias: " << state_.Bias_Gyro.transpose() << ", Accel Bias: " << state_.Bias_Acc.transpose() << std::endl;
}



void InEKFilter::SAVE_onestep_Z1(int cnt){

    if(cnt<SAVEMAXCNT)
    {
        if(textfile_flag){
            //Sending True States----------------------------------------------------------------------------
            int j=time_count + 1; // 3rd one here

            // Build rotation matrix from CSV quaternion data
            Eigen::Vector4d quat_true;
            quat_true << csvData.quat_w[j+gt_sd], csvData.quat_i[j+gt_sd], csvData.quat_j[j+gt_sd], csvData.quat_k[j+gt_sd];
            Eigen::Matrix3d temp_R = BasicFunctions_Estimator::Quaternion_to_Rotation_Matrix(quat_true);

            Eigen::Vector3d temp_eul = BasicFunctions_Estimator::Rotation_to_EulerZYX(temp_R);

            for(int i=0;i<3;i++)
            {
                for(int k=0;k<3;k++)
                {
                    SAVE_Z1[idx_TRUE_Rotation+i*3+k][cnt] = temp_R(i,k);
                }
            }


            Eigen::Vector3d v_true;
            Eigen::Matrix<double,3,1> dv_true;
            Eigen::Matrix3d R_true;

            for(int i=0;i<robot.leg_no;i++){

                v_true << csvData.vel_x[j], csvData.vel_y[j], csvData.vel_z[j];
                
                // Build R_true from CSV quaternion data (same as temp_R)
                Eigen::Vector4d quat_true_loop;
                quat_true_loop << csvData.quat_w[j], csvData.quat_i[j], csvData.quat_j[j], csvData.quat_k[j];
                R_true = BasicFunctions_Estimator::Quaternion_to_Rotation_Matrix(quat_true_loop);
                
                dv_true = v_true
                          + R_true * robot.get_foot_velocity(i+1)
                          + R_true * BasicFunctions_Estimator::Hat_so3(IMU_Gyro[frame_count])*robot.get_foot_position(i+1);


                for(int k=0; k<3; k++){

                    // if(cnt>=WINDOW_SIZE) {

                    //     SAVE_Z1[idx_ESTIMATED_dv + 3 * i + k][cnt] = d_v[frame_count](3*i + k);
                    //     SAVE_Z1[idx_TRUE_dv + 3 * i + k][cnt] = dv_true(k);
                    // }

                    SAVE_Z1[idx_ESTIMATED_dv + 3 * i + k][cnt] = d_v[frame_count](3*i + k);
                    SAVE_Z1[idx_TRUE_dv + 3 * i + k][cnt] = dv_true(k);
                }
            }



            // Save position, velocity, biases, and RPY
            for(int i=0;i<3;i++)
            {
                SAVE_Z1[idx_TRUE_Position+i][cnt] = csvData.pos_x[j+gt_sd] * (i == 0) + csvData.pos_y[j+gt_sd] * (i == 1) + csvData.pos_z[j+gt_sd] * (i == 2);
                SAVE_Z1[idx_TRUE_Velocity+i][cnt] = csvData.vel_x[j+gt_sd] * (i == 0) + csvData.vel_y[j+gt_sd] * (i == 1) + csvData.vel_z[j+gt_sd] * (i == 2);
                SAVE_Z1[idx_TRUE_Bias_Gyro+i][cnt] = 0;
                SAVE_Z1[idx_TRUE_Bias_Acc+i][cnt] = 0;
                SAVE_Z1[idx_TRUE_rpy+i][cnt] = temp_eul(i);
            }


            // Save contact states
            for(int i=0;i<robot.leg_no;i++)
            {
                SAVE_Z1[idx_TRUE_Contact+i][cnt] = (i == 0) ? csvData.lfoot_contact[j+gt_sd] : csvData.rfoot_contact[j+gt_sd];
                SAVE_Z1[idx_TRUE_Slip+i][cnt] = SLIP_t[frame_count](i);
                SAVE_Z1[idx_TRUE_Hard_Contact+i][cnt] = HARD_CONTACT_t[frame_count](i);
            }

        }else{

            ///True state saving
        }



        for(int i=0;i<robot.leg_no;i++)
        {
            SAVE_Z1[idx_ESTIMATED_Contact+i][cnt] = CONTACT_t[frame_count](i);
            SAVE_Z1[idx_ESTIMATED_Slip+i][cnt] = SLIP_t[frame_count](i);
            SAVE_Z1[idx_ESTIMATED_Hard_Contact+i][cnt] = HARD_CONTACT_t[frame_count](i);

        }



        Eigen::Vector3d temp_eul = BasicFunctions_Estimator::Rotation_to_EulerZYX(filter.getState().getRotation());
        for(int i=0;i<3;i++)
        {
            SAVE_Z1[idx_ESTIMATED_Position+i][cnt] = filter.getState().getPosition()(i);
            SAVE_Z1[idx_ESTIMATED_Velocity+i][cnt] = filter.getState().getVelocity()(i);
            SAVE_Z1[idx_ESTIMATED_Bias_Gyro+i][cnt] = filter.getState().getGyroscopeBias()(i);
            SAVE_Z1[idx_ESTIMATED_Bias_Acc+i][cnt] = filter.getState().getAccelerometerBias()(i);
            SAVE_Z1[idx_ESTIMATED_rpy+i][cnt] = temp_eul(i);
        }

        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                SAVE_Z1[idx_ESTIMATED_Rotation+i*3+j][cnt] = filter.getState().getRotation()(i,j);
            }
        }

        // {
        //     Eigen::Vector3d bg = filter.getState().getGyroscopeBias();
        //     Eigen::Vector3d ba = filter.getState().getAccelerometerBias();
        //     std::cout << "time " << cnt << " Bias_Gyro: " << bg.transpose()
        //               << " Bias_Acc: " << ba.transpose() << std::endl;
        // }


        // // Print all saved values with labels for debugging
        // std::cout << "\n=== SAVE_Z1 Values at cnt=" << cnt << " ===" << std::endl;
        // std::cout << "TRUE Position: [" << SAVE_Z1[idx_TRUE_Position+0][cnt] << ", " 
        //           << SAVE_Z1[idx_TRUE_Position+1][cnt] << ", " << SAVE_Z1[idx_TRUE_Position+2][cnt] << "]" << std::endl;
        // std::cout << "TRUE Velocity: [" << SAVE_Z1[idx_TRUE_Velocity+0][cnt] << ", " 
        //           << SAVE_Z1[idx_TRUE_Velocity+1][cnt] << ", " << SAVE_Z1[idx_TRUE_Velocity+2][cnt] << "]" << std::endl;
        // std::cout << "TRUE Bias_Gyro: [" << SAVE_Z1[idx_TRUE_Bias_Gyro+0][cnt] << ", " 
        //           << SAVE_Z1[idx_TRUE_Bias_Gyro+1][cnt] << ", " << SAVE_Z1[idx_TRUE_Bias_Gyro+2][cnt] << "]" << std::endl;
        // std::cout << "TRUE Bias_Acc: [" << SAVE_Z1[idx_TRUE_Bias_Acc+0][cnt] << ", " 
        //           << SAVE_Z1[idx_TRUE_Bias_Acc+1][cnt] << ", " << SAVE_Z1[idx_TRUE_Bias_Acc+2][cnt] << "]" << std::endl;
        // std::cout << "TRUE RPY: [" << SAVE_Z1[idx_TRUE_rpy+0][cnt] << ", " 
        //           << SAVE_Z1[idx_TRUE_rpy+1][cnt] << ", " << SAVE_Z1[idx_TRUE_rpy+2][cnt] << "]" << std::endl;
        
        // std::cout << "\nESTIMATED Position: [" << SAVE_Z1[idx_ESTIMATED_Position+0][cnt] << ", " 
        //           << SAVE_Z1[idx_ESTIMATED_Position+1][cnt] << ", " << SAVE_Z1[idx_ESTIMATED_Position+2][cnt] << "]" << std::endl;
        // std::cout << "ESTIMATED Velocity: [" << SAVE_Z1[idx_ESTIMATED_Velocity+0][cnt] << ", " 
        //           << SAVE_Z1[idx_ESTIMATED_Velocity+1][cnt] << ", " << SAVE_Z1[idx_ESTIMATED_Velocity+2][cnt] << "]" << std::endl;
        // std::cout << "ESTIMATED Bias_Gyro: [" << SAVE_Z1[idx_ESTIMATED_Bias_Gyro+0][cnt] << ", " 
        //           << SAVE_Z1[idx_ESTIMATED_Bias_Gyro+1][cnt] << ", " << SAVE_Z1[idx_ESTIMATED_Bias_Gyro+2][cnt] << "]" << std::endl;
        // std::cout << "ESTIMATED Bias_Acc: [" << SAVE_Z1[idx_ESTIMATED_Bias_Acc+0][cnt] << ", " 
        //           << SAVE_Z1[idx_ESTIMATED_Bias_Acc+1][cnt] << ", " << SAVE_Z1[idx_ESTIMATED_Bias_Acc+2][cnt] << "]" << std::endl;
        // std::cout << "ESTIMATED RPY: [" << SAVE_Z1[idx_ESTIMATED_rpy+0][cnt] << ", " 
        //           << SAVE_Z1[idx_ESTIMATED_rpy+1][cnt] << ", " << SAVE_Z1[idx_ESTIMATED_rpy+2][cnt] << "]" << std::endl;
        
        // std::cout << "\nContact States:" << std::endl;
        // for(int i=0; i<robot.leg_no; i++) {
        //     std::cout << "  Leg " << i << " - Contact: " << SAVE_Z1[idx_ESTIMATED_Contact+i][cnt] 
        //               << ", Slip: " << SAVE_Z1[idx_ESTIMATED_Slip+i][cnt] 
        //               << ", Hard_Contact: " << SAVE_Z1[idx_ESTIMATED_Hard_Contact+i][cnt] << std::endl;
        // }
        // std::cout << "================================\n" << std::endl;





    }
    else
    {
        cnt = SAVEMAXCNT-1;
        //        overcnt = true;
        cout<<"over Max SAVE_cnt!!"<<endl;
    }







}

// where previous data is moved to index 0 and new data is moved to index 1
void InEKFilter::sliding_window(){

    if (frame_count>0){
        std::swap(HARD_CONTACT_t[0],HARD_CONTACT_t[1]);
        std::swap(CONTACT_t[0],CONTACT_t[1]);
        std::swap(SLIP_t[0],SLIP_t[1]);
        std::swap(IMU_Gyro[0],IMU_Gyro[1]);
        std::swap(IMU_Acc[0],IMU_Acc[1]);
        std::swap(ENCODER[0],ENCODER[1]);
        std::swap(ENCODERDOT[0],ENCODERDOT[1]);
        d_v[0].swap(d_v[1]);
    }

}

void InEKFilter::do_SAVE_Z1_all(std::string cov_info){
    cout<<"DO SAVE_Z1 ALL"<<endl;


    FILE* ffp = NULL;
    std::string str("../result/pinnochio_inekf/");


    std::ostringstream strs,strs2,strs3;

    str = str + estimator_info +"_"+ cov_info + "_" + initial_info + "_" + file_info + "x" + std::to_string(time_count) + "_1.txt";


    ffp= fopen(str.c_str(),"w");

    for(int i=0;i<time_count;i++)
    {
        for(int j=0;j<SAVEMAX;j++)
        {

            fprintf(ffp,"%f\t",SAVE_Z1[j][i]);

        }

        fprintf(ffp,"\n");
        if(i/10000==0)
        {
            //cout<<"saving... "<<i+1<<" / "<<time_count<<endl;
        }
    }

    fclose(ffp);

    SAVE_cnt=0;

    cout<<str.c_str()<<endl;
    cout<<"*** SAVE_Z1 DONE ***"<<endl;
    //    cout<<"!!"<<endl;

    // Column Index Map (Digit robot - 2 legs):
    // TRUE_Rotation:       0-8   (9 values: 3x3 rotation matrix)
    // TRUE_Velocity:       9-11  (3 values: vx, vy, vz)
    // TRUE_Position:       12-14 (3 values: px, py, pz)
    // TRUE_dv:             15-20 (6 values: 2 legs × 3 components)
    // TRUE_Bias_Gyro:      21-23 (3 values: gyro bias)
    // TRUE_Bias_Acc:       24-26 (3 values: accel bias)
    // TRUE_Contact:        27-28 (2 values: left, right foot contact)
    // TRUE_Slip:           29-30 (2 values: left, right foot slip)
    // TRUE_Hard_Contact:   31-32 (2 values: left, right foot hard contact)
    // TRUE_rpy:            33-35 (3 values: roll, pitch, yaw)
    // EST_Rotation:        36-44 (9 values: 3x3 rotation matrix)
    // EST_Velocity:        45-47 (3 values: vx, vy, vz)
    // EST_Position:        48-50 (3 values: px, py, pz)
    // EST_dv:              51-56 (6 values: 2 legs × 3 components)
    // EST_Bias_Gyro:       57-59 (3 values: gyro bias)
    // EST_Bias_Acc:        60-62 (3 values: accel bias)
    // EST_Contact:         63-64 (2 values: left, right foot contact)
    // EST_Slip:            65-66 (2 values: left, right foot slip)
    // EST_Hard_Contact:    67-68 (2 values: left, right foot hard contact)
    // EST_rpy:             69-71 (3 values: roll, pitch, yaw)
    // Additional columns:  96-97 (ITERATION_No, TIME_PER_STEP)

}





// Inputs are 30x1 sensor measurement vector and 4x1 contact state vector
void InEKFilter::Onestep(Eigen::Matrix<double,30,1> &Sensor_i, Eigen::Matrix<bool,2,1> &Contact_i,ROBOT_STATES &state_){

    //std::cout<<endl<<endl<<"Now step "<<time_count<<" starts!"<<endl;
    clock_t start, finish;
    double duration;
    start = clock();

    new_measurement(Sensor_i, Contact_i); // read in new measurement and calculate rotation, velocity updates and slip states
    Propagate_Correct();

    send_states(state_);

    SAVE_onestep_Z1(time_count);

    finish = clock();
    duration = (double)(finish - start) / CLOCKS_PER_SEC;
    SAVE_Z1[96][time_count] = 1;
    SAVE_Z1[97][time_count] = duration;
    //cout<< "filter operating time is "<<duration<<endl<<endl;

    if(filter.getState().getVelocity().norm()>100 && dt!=0){
       cout<<"Diverged!! from time "<<time_count<<endl;
        dt = 0;
    }


    frame_count++;
    if(frame_count>1)
    {
        frame_count = 1;
        sliding_window_flag = true;
    }
    if(textfile_flag)
    {
        time_count++;
    }


    // Frame 1 information is now moved to frame 0
    sliding_window();


}

