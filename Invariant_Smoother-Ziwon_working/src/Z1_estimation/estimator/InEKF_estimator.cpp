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

void InEKFilter::Call_File(std::string file_name){

    //--------------_Call_File-------------------
    file_info = file_name;

    ifstream myfile;
    std::string sensor_data_path = "../../polished_data/" + file_name + "_sensordata.txt";
    std::string groundtruth_data_path = "../../polished_data/" + file_name + "_groundtruth.txt";
    myfile.open(sensor_data_path);
    //myfile.open("/home/zwy/Desktop/data_making_file/slippery7_sensordata_0.01.txt");
    //sensordata is sensor measurement 0~5 : IMU gyro/acc, 6~17 : 3 encoder values for 4 legs

    // Print the file name and directory for debugging
    std::cout << "Trying to open file: " << sensor_data_path << std::endl;

    char abs_path[4096];
    if (realpath(sensor_data_path.c_str(), abs_path) != NULL) {
        std::cout << "Absolute path: " << abs_path << std::endl;
    } else {
        perror("realpath() error");
    }

    if (myfile.is_open()) {
        std::cout << "yes file opened" << std::endl;
    } else {
        std::cout << "no file didnt open" << std::endl;
    }

    string line;
    string temp = "";

    string trash;
    for(int k=0; k<NUM_OF_TRASH_DATA; k++){
        std::getline(myfile, trash);
    }

    row_index = 0;
    while (std::getline(myfile, line)) { //while there is a line


        column_index = 0;
        for (int i = 0; i < line.size(); i++) { // for each character in rowstring
            if (!isblank(line[i])) { // if it is not blank, do this
                string d(1, line[i]); // convert character to string
                temp.append(d); // append the two strings
            } else {
                SensorData[row_index][column_index] = stod(temp);  // convert string to double
                temp = ""; // reset the capture
                column_index++; // increment b cause we have a new number
            }
        }

        SensorData[row_index][column_index] = stod(temp);

        if (row_index >= max_time){
            break;
        }
        temp = "";
        row_index++; // onto next row



    }
    myfile.close();
//cout<<"sensordata row, col "<<row_index<<","<<column_index<<endl;

    myfile.open(groundtruth_data_path);

    //myfile.open("/home/zwy/Desktop/data_making_file/slippery7_groundtruth_0.01.txt");
    //groundtruth is true state value (motion capture)
    //0~2 is body position, 3~6 is quaternion


    row_index = 0; // row index

    gt_sd = 8;

    string line2;
    temp = "";

    for(int k=0; k<NUM_OF_TRASH_DATA; k++){
        std::getline(myfile, trash);
    }


    while (std::getline(myfile, line2)) { //while there is a line
        column_index = 0;

        for (int i = 0; i < line2.size(); i++) { // for each character in rowstring
            if (!isblank(line2[i])) { // if it is not blank, do this
                string d(1, line2[i]); // convert character to string
                temp.append(d); // append the two strings

            } else {
                GroundTruth[row_index][column_index] = stod(temp);  // convert string to double


                temp = ""; // reset the capture
                column_index++; // increment b cause we have a new number
            }
        }


        GroundTruth[row_index][column_index] = stod(temp);
        if (row_index >= max_time){


            break;
        }
        temp = "";
        row_index++; // onto next row

    }


cout<<"groundtruth row, col "<<row_index<<","<<column_index<<endl;
    myfile.close();


    textfile_flag = true;
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

        // initialize from ground truth + initial condition offset (initial conditions aren't initial
        // conditions per se but additive offsets to ground truth)
        p0 << GroundTruth[1+gt_sd][12]+initial_condition(0),GroundTruth[1+gt_sd][13]+initial_condition(1),GroundTruth[1+gt_sd][14]+initial_condition(2);
        v0 << GroundTruth[1+gt_sd][9]+initial_condition(7),GroundTruth[1+gt_sd][10]+initial_condition(8),GroundTruth[1+gt_sd][11]+initial_condition(9);

        // These are zero for now
        bg0 << initial_condition(10), initial_condition(11), initial_condition(12);
        ba0 << initial_condition(13), initial_condition(14), initial_condition(15);

        R0 <<GroundTruth[1+gt_sd][0],GroundTruth[1+gt_sd][1],GroundTruth[1+gt_sd][2]
                ,GroundTruth[1+gt_sd][3],GroundTruth[1+gt_sd][4],GroundTruth[1+gt_sd][5]
                ,GroundTruth[1+gt_sd][6],GroundTruth[1+gt_sd][7],GroundTruth[1+gt_sd][8]; // converted from roll pitch yaw to R matrix

        Vector3d Euler = BasicFunctions_Estimator::Rotation_to_EulerZYX(R0); // converted back to euler angles to add offset
        // add initial condition offset
        Euler(0) = Euler(0)+ initial_condition[3];
        Euler(1) = Euler(1)+ initial_condition[4];
        Euler(2) = Euler(2)+ initial_condition[5];
        R0 = BasicFunctions_Estimator::EulerZYX_to_R_bw(Euler); // back to rotation matrix

    }else{

        Eigen::Vector4d initial_quaternion = initial_condition.block(3,0,4,1);
        R0=BasicFunctions_Estimator::Quaternion_to_Rotation_Matrix(initial_quaternion);;
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

//    cout << R0 << endl;
//    cout << "Initial position (x,y,z): " << p0.transpose() << endl;
//    cout << "Initial velocity (x,y,z): " << v0.transpose() << endl;

}



// Inputs are 30x1 sensor measurement vector and 4x1 contact state vector
void InEKFilter::new_measurement(Eigen::Matrix<double,30,1> &Sensor_i, Eigen::Matrix<bool,4,1> &Contact_i)
{


    //sensor measurement reading
    if(textfile_flag)
    {

        int j=time_count + 1;

        IMU_Gyro[frame_count] << SensorData[j][0],SensorData[j][1],SensorData[j][2]; // Save from sensor file
        IMU_Acc[frame_count] << SensorData[j][3],SensorData[j][4],SensorData[j][5]; // Save from sensor file

        ENCODER[frame_count] <<SensorData[j][6],SensorData[j][7],SensorData[j][8],SensorData[j][9],SensorData[j][10],SensorData[j][11],
                SensorData[j][12],SensorData[j][13],SensorData[j][14],SensorData[j][15],SensorData[j][16],SensorData[j][17]; // 12 encoder values for 4 legs (3 each)

        ENCODERDOT[frame_count] <<SensorData[j][18],SensorData[j][19],SensorData[j][20],SensorData[j][21],SensorData[j][22],SensorData[j][23],
                SensorData[j][24],SensorData[j][25],SensorData[j][26],SensorData[j][27],SensorData[j][28],SensorData[j][29];

        CONTACT_t[frame_count] << GroundTruth[j][19],GroundTruth[j][20],GroundTruth[j][21],GroundTruth[j][22]; // 4 contact states for 4 legs (last columns of ground truth file)

        imu_measurement <<IMU_Gyro[frame_count], IMU_Acc[frame_count];

    }else{

        IMU_Gyro[frame_count] = Sensor_i.block(0,0,3,1);
        IMU_Acc[frame_count] = Sensor_i.block(3,0,3,1);

        ENCODER[frame_count] = Sensor_i.block(6,0,12,1);
        //ENCODER[frame_count] << Sensor_i.block(6,0,3,1),Sensor_i.block(6,0,3,1),Sensor_i.block(6,0,3,1),Sensor_i.block(6,0,3,1);

        ENCODERDOT[frame_count] =Sensor_i.block(18,0,12,1);
        CONTACT_t[frame_count] = Contact_i;

        imu_measurement <<IMU_Gyro[frame_count], IMU_Acc[frame_count];

    }

    //propagate-----------------------------------------------------------------------------

    //Slip Rejection-------------------------------------------------------------------------------

    if ( frame_count==0 ){

        for (int k=0; k<4; k++){
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

        for(int p=1; p<=1; p++){ // originally kept for a larger window size, but now just 1

            // Compute FK and Jacobian for all legs at once
            Eigen::Matrix<double, 12, 1> all_foot_positions = robot.Forward_Kinematics_All_Legs(ENCODER[p]);
            Eigen::Matrix<double, 12, 12> all_jacobians = robot.Jacobian_All_Legs(ENCODER[p]);
            
            for (int k=0; k<robot.leg_no; k++) {

                // Extract 3x3 Jacobian block and 3x1 foot position for this leg
                Eigen::Matrix<double, 3, 3> J_leg = all_jacobians.block<3, 3>(3*k, 3*k);
                Eigen::Vector3d foot_pos = all_foot_positions.segment<3>(3*k);
                
                d_v[p].block(3*k,0,3,1) = Velocity_s[p]
                                          + Rotation_s[p]*J_leg*ENCODERDOT[p].block<3,1>(3*k,0)
                                          + Rotation_s[p]*BasicFunctions_Estimator::Hat_so3(IMU_Gyro[p] - Bias_Gyro_s[p])*foot_pos;

                HARD_CONTACT_t[p](k)  = CONTACT_t[p](k);

                if ((slip_rejection_mode == true) && (CONTACT_t[p](k) == true) &&
                    (d_v[p].block(3 * k, 0, 3, 1).norm() > robot.slip_threshold)) {
                    SLIP_t[p](k) = true;
                }

//            HARD_CONTACT_t[1](k)  = CONTACT_t[1](k)-SLIP_t[1](k);
            }
        }




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

        for (int i=0; i<4; i++) {
            int id = i;
            bool indicator = HARD_CONTACT_t[frame_count](i);

            contacts.push_back(pair<int,bool> (id, indicator));
        }

    //cout<<"HC "<<HARD_CONTACT_t[frame_count]<<endl;
    filter.setContacts(contacts);

    // Compute FK and Jacobian for all legs at once
    Eigen::Matrix<double, 12, 1> all_foot_positions_fc = robot.Forward_Kinematics_All_Legs(ENCODER[frame_count]);
    Eigen::Matrix<double, 12, 12> all_jacobians_fc = robot.Jacobian_All_Legs(ENCODER[frame_count]);

    int id;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double,6,6> covariance;
    inekf::vectorKinematics measured_kinematics;
    Eigen::Matrix3d Covariance_Encoder_leg = robot.cov_enc_const * Eigen::Matrix3d::Identity();

    // Build measured kinematics from batch results
    for (int k=0; k<4; k++) {
        id = k;
        pose.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
        pose.block<3,1>(0,3) = all_foot_positions_fc.segment<3>(3*k); // Extract foot position for this leg
        
        // Extract Jacobian for this leg and compute measurement covariance
        Eigen::Matrix3d FK_Jacobian = all_jacobians_fc.block<3, 3>(3*k, 3*k);
        covariance.block(3,3, 3,3) = FK_Jacobian * Covariance_Encoder_leg * FK_Jacobian.transpose();

        inekf::Kinematics frame(id, pose, covariance);
        measured_kinematics.push_back(frame);
    }
    // Correct state using kinematic measurements

    bool flag = true; // should be false for loop below to work properly
    if(time_count%10==5){ // correct every 10th step from 5, 15, 25, ..etc
        flag = true;
    }

    filter.CorrectKinematics(measured_kinematics, flag);

    // Compute FK and Jacobian for all legs at once
    Eigen::Matrix<double, 12, 1> all_foot_positions = robot.Forward_Kinematics_All_Legs(ENCODER[1]);
    Eigen::Matrix<double, 12, 12> all_jacobians = robot.Jacobian_All_Legs(ENCODER[1]);
    
    for (int k=0; k<robot.leg_no; k++){
        // Extract 3x3 Jacobian block and 3x1 foot position for this leg
        Eigen::Matrix<double, 3, 3> J_leg = all_jacobians.block<3, 3>(3*k, 3*k);
        Eigen::Vector3d foot_pos = all_foot_positions.segment<3>(3*k);
        
        d_v[1].block(3*k,0,3,1) = filter.getState().getVelocity()
                + filter.getState().getRotation()*J_leg*ENCODERDOT[1].block<3,1>(3*k,0)
                + filter.getState().getRotation()*BasicFunctions_Estimator::Hat_so3(IMU_Gyro[1] - filter.getState().getGyroscopeBias())*foot_pos;


        //Test Junny
        SLIP_t[1](k) = false;

        if (slip_rejection_mode == true && CONTACT_t[1](k) == true &&
            d_v[1].block(3*k,0, 3,1).norm() > slip_threshold){
            SLIP_t[1](k) = true;
        }
        HARD_CONTACT_t[1](k)  = CONTACT_t[1](k);

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



}



void InEKFilter::SAVE_onestep_Z1(int cnt){

    if(cnt<SAVEMAXCNT)
    {
        if(textfile_flag){
            //Sending True States----------------------------------------------------------------------------
            int j=time_count + 1;

            Eigen::Matrix3d temp_R;
            temp_R << GroundTruth[j+gt_sd][0],GroundTruth[j+gt_sd][1],GroundTruth[j+gt_sd][2]
                    ,GroundTruth[j+gt_sd][3],GroundTruth[j+gt_sd][4],GroundTruth[j+gt_sd][5]
                    ,GroundTruth[j+gt_sd][6],GroundTruth[j+gt_sd][7],GroundTruth[j+gt_sd][8];

            Eigen::Vector3d temp_eul = BasicFunctions_Estimator::Rotation_to_EulerZYX(temp_R);

            for(int i=0;i<3;i++)
            {
                for(int k=0;k<3;k++)
                {
                    SAVE_Z1[idx_TRUE_Rotation+i*3+k][cnt] = GroundTruth[j+gt_sd][3*i+k];
                }
            }


            Eigen::Vector3d v_true;
            Eigen::Matrix<double,3,1> dv_true;
            Eigen::Matrix3d R_true;

            // Compute FK and Jacobian for all legs at once
            Eigen::Matrix<double, 12, 1> all_foot_positions = robot.Forward_Kinematics_All_Legs(ENCODER[frame_count]);
            Eigen::Matrix<double, 12, 12> all_jacobians = robot.Jacobian_All_Legs(ENCODER[frame_count]);

            for(int i=0;i<robot.leg_no;i++){

                v_true << GroundTruth[time_count+1][9], GroundTruth[time_count+1][10], GroundTruth[time_count+1][11];
                R_true << GroundTruth[time_count+1][0], GroundTruth[time_count+1][1], GroundTruth[time_count+1][2],
                        GroundTruth[time_count+1][3], GroundTruth[time_count+1][4], GroundTruth[time_count+1][5],
                        GroundTruth[time_count+1][6], GroundTruth[time_count+1][7], GroundTruth[time_count+1][8];

                // Extract 3x3 Jacobian block and 3x1 foot position for this leg
                Eigen::Matrix<double, 3, 3> J_leg = all_jacobians.block<3, 3>(3*i, 3*i);
                Eigen::Vector3d foot_pos = all_foot_positions.segment<3>(3*i);
                
                dv_true = v_true
                          + R_true * J_leg*ENCODERDOT[frame_count].block(3*i,0, 3,1)
                          + R_true * BasicFunctions_Estimator::Hat_so3(IMU_Gyro[frame_count])*foot_pos;


                for(int k=0; k<3; k++){

                    if(cnt>=WINDOW_SIZE) {

                        SAVE_Z1[idx_ESTIMATED_dv + 3 * i + k][cnt] = d_v[frame_count](3*i + k);
                        SAVE_Z1[idx_TRUE_dv + 3 * i + k][cnt] = dv_true(k);
                    }
                }
            }



            for(int i=0;i<3;i++)
            {
                SAVE_Z1[idx_TRUE_Position+i][cnt] = GroundTruth[j+gt_sd][12+i];
                SAVE_Z1[idx_TRUE_Velocity+i][cnt] = GroundTruth[j+gt_sd][9+i];
                SAVE_Z1[idx_TRUE_Bias_Gyro+i][cnt] =0;
                SAVE_Z1[idx_TRUE_Bias_Acc+i][cnt] = 0;
                SAVE_Z1[idx_TRUE_rpy+i][cnt] = temp_eul(i);
            }


            for(int i=0;i<robot.leg_no;i++)
            {
                SAVE_Z1[idx_TRUE_Contact+i][cnt] = GroundTruth[j+gt_sd][19+i];
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
    std::string str("../result/");


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

    // Print a short summary of the column categories written to the result file
    cout << "Saved categories in file (column groups in order): "
         << "TRUE_Rotation(9), TRUE_Velocity(3), TRUE_Position(3), TRUE_dv(12), TRUE_Bias_Gyro(3), TRUE_Bias_Acc(3), TRUE_Contact(4), TRUE_Slip(4), TRUE_Hard_Contact(4), TRUE_rpy(3), "
         << "EST_Rotation(9), EST_Velocity(3), EST_Position(3), EST_dv(12), EST_Bias_Gyro(3), EST_Bias_Acc(3), EST_Contact(4), EST_Slip(4), EST_Hard_Contact(4), EST_rpy(3), "
         << "ITERATION_No(1), BACKPPGN_No(1), TIME_PER_STEP(1), COST(4), JAC_STD(1)" << endl;

}





// Inputs are 30x1 sensor measurement vector and 4x1 contact state vector
void InEKFilter::Onestep(Eigen::Matrix<double,30,1> &Sensor_i, Eigen::Matrix<bool,4,1> &Contact_i,ROBOT_STATES &state_){

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

