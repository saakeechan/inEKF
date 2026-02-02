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
#include <eigen3/Eigen/Dense>
#include <vector>
#include "InEKF_estimator.hpp"
#include "../utility/Digit/csvParserDigit.hpp"

using std::cout;  
using std::endl;

// This is just reading of the sensor data file and ground truth data file

void InEKFilter::LoadCSVData(const std::string& csv_filename) {
    // Load CSV data using digit parser
    csvData = ParseDigitCSV(csv_filename);
    std::cout << "Loaded CSV data with " << csvData.timestamp.size() << " rows." << std::endl;
    textfile_flag  = true;
}

std::vector<double> InEKFilter::Variable_Contact_Cov(int time){ // time is either 0 or 1 cause sliding window of 2 for filter; fix this for smoother later

    std::vector<double> contact_cov_array;

    for(int k=0; k<robot.num_legs; k++){

        double contact_cov = robot.cov_contact_const;

        //  only legs with actual contact get potentially adjusted covariance.
        //  Swing state legs get assingined a default value for consistent index purposes
        if(HARD_CONTACT_t[time][k]){ 

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





void InEKFilter::Initialize(double dt_arg, Eigen::Matrix<double,12,1> cov_val_setting, Eigen::Matrix<double, 16, 1> initial_condition, int sliding_window_size_arg)
{

    //  ---- Initialize invariant extended Kalman filter ----- //

    filter.Reset();

    frame_count = 0; // Initialize frame counter
    time_count = 0; // Initialize time counter
    
    // Set sliding window size and allocate buffers
    int SLIDING_WINDOW_SIZE = sliding_window_size_arg;
    std::cout << "Sliding window size set to: " << SLIDING_WINDOW_SIZE << std::endl;
    dt = dt_arg;
    
    // Resize all sliding window vectors
    Position_s.resize(SLIDING_WINDOW_SIZE);
    Rotation_s.resize(SLIDING_WINDOW_SIZE);
    Velocity_s.resize(SLIDING_WINDOW_SIZE);
    d_v.resize(SLIDING_WINDOW_SIZE);
    Bias_Acc_s.resize(SLIDING_WINDOW_SIZE);
    Bias_Gyro_s.resize(SLIDING_WINDOW_SIZE);
    CONTACT_t.resize(SLIDING_WINDOW_SIZE);
    HARD_CONTACT_t.resize(SLIDING_WINDOW_SIZE);
    SLIP_t.resize(SLIDING_WINDOW_SIZE); // think about changing to Slip_Contact_t
    IMU_Gyro.resize(SLIDING_WINDOW_SIZE);
    IMU_Acc.resize(SLIDING_WINDOW_SIZE);
    ENCODER.resize(SLIDING_WINDOW_SIZE);
    ENCODERDOT.resize(SLIDING_WINDOW_SIZE);
    
    // Initialize each element to the correct size
    for (int i = 0; i < SLIDING_WINDOW_SIZE; i++) {
        // Fixed-size types
        Position_s[i] = Eigen::Vector3d::Zero();
        Rotation_s[i] = Eigen::Matrix3d::Identity();
        Velocity_s[i] = Eigen::Vector3d::Zero();
        Bias_Gyro_s[i] = Eigen::Vector3d::Zero();
        Bias_Acc_s[i] = Eigen::Vector3d::Zero();
        IMU_Gyro[i] = Eigen::Vector3d::Zero();
        IMU_Acc[i] = Eigen::Vector3d::Zero();
        
        // Dynamic-size types based on robot model
        ENCODER[i] = Eigen::VectorXd::Zero(robot.encoder_size);
        ENCODERDOT[i] = Eigen::VectorXd::Zero(robot.encoderdot_size);
        d_v[i] = Eigen::VectorXd::Zero(3 * robot.num_legs);  // 3 components per leg
        
        // Contact state vectors - resize inner vector for each leg
        CONTACT_t[i].resize(robot.num_legs, false);
        HARD_CONTACT_t[i].resize(robot.num_legs, false);
        SLIP_t[i].resize(robot.num_legs, false);
    }
        
    robot.SetCovariances(cov_val_setting); // Initialize covariance of the 12x1 vector input


    std::string mode;
    if(variable_contact_cov_mode == true){
        mode="(CS" + BasicFunctions_Estimator::to_string_n_signficant_figures(cov_amplifier,2)+ ")";
    }else if(slip_rejection_mode == true){
        mode="(SR" + BasicFunctions_Estimator::to_string_n_signficant_figures(slip_threshold,2) + ")";
    }

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

        int idx = 0 + CSV_Skip_Data; // To skip trash data in CSV at the beginning

        // initialize from CSV data + initial condition offset because we might want to start from non-zero state as described in the CSV file

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

    }else{ // Initialize for sensor readings

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

    // Initial State has initial states + prior covariance


    imu_measurement = Eigen::Matrix<double,6,1>::Zero();
    imu_measurement_prev = Eigen::Matrix<double,6,1>::Zero();

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
}


// Inputs are 30x1 sensor measurement vector and dynamic contact state vector
void InEKFilter::new_measurement(Eigen::Matrix<double,30,1> &Sensor_i, std::vector<bool> &Contact_i)
{
    // ========================================================================
    // ROBOT-SPECIFIC ENCODER ASSIGNMENT
    // This section assigns encoder and encoder velocity data from CSV to 
    // the appropriate indices for Pinocchio FK.
    // 
    // When creating a new robot model:
    // 1. Implement assign_encoder_from_csv() in your Robot_Model_XXX class
    // 2. Implement assign_encoderdot_from_csv() in your Robot_Model_XXX class
    // 3. These methods should map CSV data to correct joint indices per your URDF/MJCF
    // ========================================================================


    //sensor measurement reading
    if(textfile_flag) // Read from CSV file
    {

        int j = time_count + CSV_Skip_Data; // To skip trash data in CSV at the beginning

        // IMU data from CSV (gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z)
        IMU_Gyro[frame_count] << csvData.gyro_x[j], csvData.gyro_y[j], csvData.gyro_z[j];
        IMU_Acc[frame_count] << csvData.acc_x[j], csvData.acc_y[j], csvData.acc_z[j];

        // Map CSV data to encoder vectors using robot-specific indices
        robot.map_csv_to_encoders(csvData.get_active_encoders(j), ENCODER[frame_count], "encoder");
        robot.map_csv_to_encoders(csvData.get_active_encoderdots(j), ENCODERDOT[frame_count], "encoderdot");

        // Get contact state from CSV
        CONTACT_t[frame_count] = csvData.get_contact_state(j);


        imu_measurement << IMU_Gyro[frame_count], IMU_Acc[frame_count]; // Consolidate IMU measurement because propagate function needs it

    }else{ // Read from Sensors

        IMU_Gyro[frame_count] = Sensor_i.block(0,0,3,1);
        IMU_Acc[frame_count] = Sensor_i.block(3,0,3,1);

        ENCODER[frame_count] = Sensor_i.block(6,0,12,1);
        //ENCODER[frame_count] << Sensor_i.block(6,0,3,1),Sensor_i.block(6,0,3,1),Sensor_i.block(6,0,3,1),Sensor_i.block(6,0,3,1);

        ENCODERDOT[frame_count] =Sensor_i.block(18,0,12,1);
        CONTACT_t[frame_count] = Contact_i;

        // have to write a robot-specific function to map Sensor_i to ENCODER and ENCODERDOT if sizes differ
        
        imu_measurement <<IMU_Gyro[frame_count], IMU_Acc[frame_count];

    }

    // Pinnocchio FK and Jacobians - compute forward kinematics and joint Jacobians as soon as you get new measurement
    robot.Forward_Kinematics(ENCODER[frame_count], ENCODERDOT[frame_count]);
    robot.compute_joint_jacobians();

    //propagate-----------------------------------------------------------------------------

    //Slip Rejection-------------------------------------------------------------------------------

    if ( frame_count==0 ){

        for (int k=0; k<robot.num_legs; k++){
            SLIP_t[0][k] = false; // Assume no legs are slipping at first frame
            HARD_CONTACT_t[0][k]  = CONTACT_t[0][k]-SLIP_t[0][k]; // Actual hard contacts are contact - slip - [true - false = true, true - true = false]
        }

    }else{

        Eigen::Vector3d phi = (imu_measurement.block(0,0, 3,1)-filter.getState().getGyroscopeBias())*dt; // (angular increment - gyro bias)(*dt)
        Rotation_s[frame_count] = filter.getState().getRotation() * BasicFunctions_Estimator::Expm_Vec(phi); // Rotation update with gyro bias correction
        JacobiSVD<Matrix3d> svd(Rotation_s[frame_count], ComputeFullU|ComputeFullV); // Cause we're doing dt, we need to re-orthogonalize the rotation matrix back to its manifold
        Rotation_s[frame_count] = svd.matrixU() * svd.matrixV().transpose();

        Velocity_s[frame_count] = filter.getState().getVelocity() + (Rotation_s[frame_count]*(imu_measurement.block(3,0, 3,1)-filter.getState().getAccelerometerBias()) + gravity)*dt;
        // Velocity update with accel bias correction and gravity

    }

        for (int k=0; k<robot.num_legs; k++) {

            d_v[frame_count].block(3*k,0,3,1) = Velocity_s[frame_count]
                                        + Rotation_s[frame_count]*robot.get_foot_velocity(k+1)
                                        + Rotation_s[frame_count]*BasicFunctions_Estimator::Hat_so3(IMU_Gyro[frame_count] - Bias_Gyro_s[frame_count])*robot.get_foot_position(k+1);

            // Initialize slip to false, then check for slip condition
            // SLIP_t[frame_count][k] = false;
            HARD_CONTACT_t[frame_count][k]  = CONTACT_t[frame_count][k];
            if ((slip_rejection_mode == true) && (CONTACT_t[frame_count][k] == true) &&
                (d_v[frame_count].block(3 * k, 0, 3, 1).norm() > slip_threshold)) {
                SLIP_t[frame_count][k] = true;
            }

            // Remove slipping legs from contact list
            // HARD_CONTACT_t[frame_count][k] = CONTACT_t[frame_count][k] - SLIP_t[frame_count][k];
        }

}



void InEKFilter::Propagate_and_Correct()

{

    if(frame_count>0) {

        filter.Propagate(imu_measurement_prev, dt, Variable_Contact_Cov(0)); // propagate with previous imu measurement and dt and also the contact covariance for static foot assumption

    }

    // change imu measurement_prev after inputting to propagation
    imu_measurement_prev = imu_measurement;


    // // Build contact constraint list from slip-filtered hard contacts
    // std::vector<pair<int,bool>> contactConstraints;
    // for (int i=0; i<robot.num_legs; i++) {
    //     contactConstraints.emplace_back(i, HARD_CONTACT_t[frame_count][i]); // instead of push_back - slightly more efficient
    // }

    // // Build contact constraint list from slip-filtered hard contacts
    std::vector<pair<int,bool>> contactConstraints;
    for (int i=0; i<robot.num_legs; i++) {
        contactConstraints.emplace_back(i, HARD_CONTACT_t[frame_count][i]); // instead of push_back - slightly more efficient
    }

    //cout<<"HC "<<HARD_CONTACT_t[frame_count]<<endl;
    filter.setContacts(contactConstraints);


    int id;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double,6,6> covariance;
    inekf::vectorKinematics measured_kinematics;
    

    // Build measured kinematics from batch results
    for (int k=0; k<robot.num_legs; k++) {
        id = k;
        pose.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
        pose.block<3,1>(0,3) = robot.get_foot_position(k+1);
        
        // Extract Jacobian for this leg and compute measurement covariance
        Eigen::MatrixXd FK_Jacobian_full = robot.get_foot_jacobian(k+1);

        // Extract only the active encoder columns (removes base DOFs + unused joints)
        Eigen::MatrixXd FK_Jacobian = robot.extract_active_encoder_jacobian(FK_Jacobian_full);
        
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

    filter.CorrectKinematics(measured_kinematics, true); // Check later to see if its okay to correct on first step itself
    
    if (frame_count != 0){ 

        for (int k=0; k<robot.num_legs; k++){
            
            d_v[frame_count].block(3*k,0,3,1) = filter.getState().getVelocity()
                    + filter.getState().getRotation()*robot.get_foot_velocity(k+1)
                    + filter.getState().getRotation()*BasicFunctions_Estimator::Hat_so3(IMU_Gyro[frame_count] - filter.getState().getGyroscopeBias())*robot.get_foot_position(k+1);

            SLIP_t[frame_count][k] = false;
            if (slip_rejection_mode == true && CONTACT_t[frame_count][k] == true &&
                d_v[frame_count].block(3*k,0, 3,1).norm() > slip_threshold){
                SLIP_t[frame_count][k] = true;
            }
            // HARD_CONTACT_t[frame_count][k]  = CONTACT_t[frame_count][k] - SLIP_t[frame_count][k];
            HARD_CONTACT_t[frame_count][k]  = CONTACT_t[frame_count][k];

        }
    }

}


void InEKFilter::send_states(ROBOT_STATES &state_){ // don't even know if I need this function


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
        state_.d_v = d_v[frame_count-1]; // create a small class/struct called contact states to send all 3 contact, slip, hard contact together
    }
}



void InEKFilter::SAVE_onestep(int save_index){

    if(save_index<SAVEMAXCNT)
    {
        if(textfile_flag){
            //Sending True States----------------------------------------------------------------------------
            int j = time_count + CSV_Skip_Data; // 3rd one here

            // Build rotation matrix from CSV quaternion data
            Eigen::Vector4d quat_true;
            quat_true << csvData.quat_w[j], csvData.quat_i[j], csvData.quat_j[j], csvData.quat_k[j];
            Eigen::Matrix3d temp_R = BasicFunctions_Estimator::Quaternion_to_Rotation_Matrix(quat_true);

            Eigen::Vector3d temp_eul = BasicFunctions_Estimator::Rotation_to_EulerZYX(temp_R);

            for(int i=0;i<3;i++)
            {
                for(int k=0;k<3;k++)
                {
                    SAVE_ROW[idx_TRUE_Rotation+i*3+k][save_index] = temp_R(i,k);
                }
            }


            Eigen::Vector3d v_true;
            Eigen::Vector<double,3> dv_true;
            Eigen::Matrix3d R_true;

            for(int i=0;i<robot.num_legs;i++){

                v_true << csvData.vel_x[j], csvData.vel_y[j], csvData.vel_z[j];
                
                // Build R_true from CSV quaternion data (same as temp_R)
                Eigen::Vector4d quat_true_loop;
                quat_true_loop << csvData.quat_w[j], csvData.quat_i[j], csvData.quat_j[j], csvData.quat_k[j];
                R_true = BasicFunctions_Estimator::Quaternion_to_Rotation_Matrix(quat_true_loop);
                
                dv_true = v_true
                          + R_true * robot.get_foot_velocity(i+1)
                          + R_true * BasicFunctions_Estimator::Hat_so3(IMU_Gyro[frame_count])*robot.get_foot_position(i+1);


                for(int k=0; k<3; k++){

                    SAVE_ROW[idx_ESTIMATED_dv + 3 * i + k][save_index] = d_v[frame_count](3*i + k);
                    SAVE_ROW[idx_TRUE_dv + 3 * i + k][save_index] = dv_true(k);
                }
            }



            // Save position, velocity, biases, and RPY
            for(int i=0;i<3;i++)
            {
                SAVE_ROW[idx_TRUE_Position+i][save_index] = csvData.pos_x[j] * (i == 0) + csvData.pos_y[j] * (i == 1) + csvData.pos_z[j] * (i == 2);
                SAVE_ROW[idx_TRUE_Velocity+i][save_index] = csvData.vel_x[j] * (i == 0) + csvData.vel_y[j] * (i == 1) + csvData.vel_z[j] * (i == 2);
                SAVE_ROW[idx_TRUE_Bias_Gyro+i][save_index] = 0;
                SAVE_ROW[idx_TRUE_Bias_Acc+i][save_index] = 0;
                SAVE_ROW[idx_TRUE_rpy+i][save_index] = temp_eul(i);
            }


            // Save contact states
            std::vector<bool> temp_contact = csvData.get_contact_state(j);
            
            for(int i=0;i<robot.num_legs;i++)
            {
                SAVE_ROW[idx_TRUE_Contact+i][save_index] = temp_contact[i];
                SAVE_ROW[idx_TRUE_Slip+i][save_index] = SLIP_t[frame_count][i];
                SAVE_ROW[idx_TRUE_Hard_Contact+i][save_index] = HARD_CONTACT_t[frame_count][i];
            }

        }else{

            // I dont know nwhat to do for sensor derived state estimation
        }



        for(int i=0;i<robot.num_legs;i++)
        {
            SAVE_ROW[idx_ESTIMATED_Contact+i][save_index] = CONTACT_t[frame_count][i];
            SAVE_ROW[idx_ESTIMATED_Slip+i][save_index] = SLIP_t[frame_count][i];
            SAVE_ROW[idx_ESTIMATED_Hard_Contact+i][save_index] = HARD_CONTACT_t[frame_count][i];

        }



        Eigen::Vector3d temp_eul = BasicFunctions_Estimator::Rotation_to_EulerZYX(filter.getState().getRotation());
        for(int i=0;i<3;i++)
        {
            SAVE_ROW[idx_ESTIMATED_Position+i][save_index] = filter.getState().getPosition()(i);
            SAVE_ROW[idx_ESTIMATED_Velocity+i][save_index] = filter.getState().getVelocity()(i);
            SAVE_ROW[idx_ESTIMATED_Bias_Gyro+i][save_index] = filter.getState().getGyroscopeBias()(i);
            SAVE_ROW[idx_ESTIMATED_Bias_Acc+i][save_index] = filter.getState().getAccelerometerBias()(i);
            SAVE_ROW[idx_ESTIMATED_rpy+i][save_index] = temp_eul(i);
        }

        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                SAVE_ROW[idx_ESTIMATED_Rotation+i*3+j][save_index] = filter.getState().getRotation()(i,j);
            }
        }

    }
    else
    {
        save_index = SAVEMAXCNT-1;
        cout<<"over Max_SAVE_cnt!!"<<endl;
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
        std::swap(d_v[0], d_v[1]);
    }

}

void InEKFilter::writeSavedData(std::string cov_info){

    // Build filename efficiently
    std::string filename = "../result/pinnochio_inekf/" + estimator_info + "_" + cov_info + "_" + 
                          initial_info + "_" + file_info + "x" + std::to_string(time_count) + "_1.txt";

    // Use C++ file stream with RAII and error checking
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "Error: Failed to open file: " << filename << std::endl;
        return;
    }

    // Write data with proper formatting
    for(int i = 0; i < time_count; i++) {
        for(int j = 0; j < SAVEMAXDATA; j++) {
            outfile << SAVE_ROW[j][i] << "\t";
        }
        outfile << "\n";
    }

    // File automatically closed by destructor
    cout << filename << endl;
    cout << "Saving to a file done" << endl;

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
void InEKFilter::Onestep(Eigen::Matrix<double,30,1> &Sensor_i, std::vector<bool> &Contact_i,ROBOT_STATES &state_){

    // clock_t start, finish;
    // double duration;
    // start = clock();

    new_measurement(Sensor_i, Contact_i); // read in new measurement and calculate rotation, velocity updates and slip states


    Propagate_and_Correct();

    // send_states(state_); Can delete this function later if not needed


    SAVE_onestep(time_count);

    // finish = clock();
    // duration = (double)(finish - start) / CLOCKS_PER_SEC;
    // SAVE_ROW[96][time_count] = 1;
    // SAVE_ROW[97][time_count] = duration;
    //cout<< "filter operating time is "<<duration<<endl<<endl;

    if(filter.getState().getVelocity().norm()>10 && dt!=0){
       cout<<"Diverged!! from time "<<time_count<<endl;
        dt = 0;
    }


    // Keep frame_count in [0, 1] range for sliding window
    frame_count++;
    if(frame_count > 1) {
        frame_count = 1;
    }
    
    if(textfile_flag) {
        time_count++;
    }

    // Frame 1 information is now moved to frame 0
    sliding_window();


}

