//
// Created by Junny on  2020-06-29.
// Last Update on       2020-06-30
//

#ifndef SRC_InEKF_estimator_HPP
#define SRC_InEKF_estimator_HPP

// Libraries
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <iostream>
#include <fstream>



// CustomBasics

#include "../InEKF/InEKF.h"
#include "../utility/BasicFunctions_Estimator.hpp"
#include "../utility/Robot_Model_Digit.hpp"
#include "DigitCSVData.hpp"


using namespace Eigen;
using namespace std;

// Robot State Type Definition
typedef struct _ROBOT_STATES_
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix3d Rotation;
    Eigen::Vector3d Velocity;
    Eigen::Vector3d Position;

    Eigen::Matrix<double, 6, 1> d;
    Eigen::Matrix<double, 6, 1> d_v;

    Eigen::Vector3d Bias_Gyro;
    Eigen::Vector3d Bias_Acc;

    Eigen::Matrix<bool, 2, 1> Contact;
    Eigen::Matrix<bool, 2, 1> Hard_Contact;
    Eigen::Matrix<bool, 2, 1> Slip;

    int contact_leg_num;

    int state_size;
    int state_idx;
    int para_size;
    int para_idx;

}ROBOT_STATES;

class InEKFilter {


public:
    //functions
    void LoadCSVData(const std::string& csv_filename);
    void Initialize(double _dt, Eigen::Matrix<double,12,1> cov_val_setting, Eigen::Matrix<double, 16, 1> initial_condition);

    void new_measurement(Eigen::Matrix<double,30,1> &Sensor_i, Eigen::Matrix<bool,2,1> &Contact_i);

    void Propagate_Correct();
    std::vector<double> Variable_Contact_Cov(int time);

    void send_states(ROBOT_STATES &state_);
    void sliding_window();

    void SAVE_onestep_Z1(int cnt);

    void Onestep(Eigen::Matrix<double,30,1> &Sensor_i, Eigen::Matrix<bool,2,1> &Contact_i,ROBOT_STATES &state_);

    void do_SAVE_Z1_all(std::string cov_info);


int NUM_OF_TRASH_DATA = 1;


    //Necessary classes & variables
    Robot_Model_Digit robot; // This is where robot model is defined. Should change this when we redfine with pinnochio

    bool sliding_window_flag=false;
    int frame_count=0;
    int time_count=0;

    double dt = robot.dt; // From Robot_Model_mini_cheetah
    Vector3d gravity;



    //Call_FILE PARAMETER
    int gt_sd = 0;
    const static int MAX_FILE_COUNT =140000;
    // double SensorData       [MAX_FILE_COUNT][30];
    // double GroundTruth      [MAX_FILE_COUNT][27];
    const static int max_time = MAX_FILE_COUNT-1;
    int row_index = 0; // row index
    int column_index = 0; // column index


    //Inner variable
    inekf::InEKF filter;

    Eigen::Matrix<double,6,1> imu_measurement;
    Eigen::Matrix<double,6,1> imu_measurement_prev;
    // Estimated State Values Window Buffer
    Vector3d                   Position_s       [2];
    Matrix3d                   Rotation_s       [2];
    Vector3d                   Velocity_s       [2];
    Eigen::Matrix<double,6,1> d_v              [2];

    Vector3d                   Bias_Acc_s       [2];
    Vector3d                   Bias_Gyro_s      [2];
    Matrix<double, 6,1>        Bias_s           [2];


    // True Values and measurement Buffer
    Eigen::Matrix<bool, 2,1>   HARD_CONTACT_t  [2];
    Eigen::Matrix<bool, 2,1>   CONTACT_t       [2];
    Eigen::Matrix<bool, 2,1>   SLIP_t          [2];

    Eigen::Matrix<double, 3,1>         IMU_Gyro         [2];
    Eigen::Matrix<double, 3,1>         IMU_Acc          [2];
    Eigen::Matrix<double,61,1>         ENCODER          [2];
    Eigen::Matrix<double,54,1>         ENCODERDOT       [2];



    //SAVE PARAMETER
    std::string estimator_info;
    std::string file_info;
    std::string initial_info;
    std::string time_size;
    std::string est_size;

    const static int SAVEMAX = 100;
    const static int SAVEMAXCNT = MAX_FILE_COUNT;
    double SAVE_Z1 [SAVEMAX][SAVEMAXCNT];
    int SAVE_cnt = 0;

    int idx_TRUE_Rotation           = 0;
    int idx_TRUE_Velocity           = idx_TRUE_Rotation + 9;
    int idx_TRUE_Position           = idx_TRUE_Velocity + 3;
    int idx_TRUE_dv                  = idx_TRUE_Position + 3;
    int idx_TRUE_Bias_Gyro          = idx_TRUE_dv + 6;
    int idx_TRUE_Bias_Acc           = idx_TRUE_Bias_Gyro + 3;
    int idx_TRUE_Contact            = idx_TRUE_Bias_Acc + 3;
    int idx_TRUE_Slip               = idx_TRUE_Contact + 2;
    int idx_TRUE_Hard_Contact       = idx_TRUE_Slip+ 2;
    int idx_TRUE_rpy                = idx_TRUE_Hard_Contact + 2;

    int idx_ESTIMATED_Rotation      = idx_TRUE_rpy + 3;
    int idx_ESTIMATED_Velocity      = idx_ESTIMATED_Rotation + 9;
    int idx_ESTIMATED_Position      = idx_ESTIMATED_Velocity + 3;
    int idx_ESTIMATED_dv             = idx_ESTIMATED_Position + 3;
    int idx_ESTIMATED_Bias_Gyro     = idx_ESTIMATED_dv + 6;
    int idx_ESTIMATED_Bias_Acc      = idx_ESTIMATED_Bias_Gyro + 3;
    int idx_ESTIMATED_Contact       = idx_ESTIMATED_Bias_Acc + 3;
    int idx_ESTIMATED_Slip          = idx_ESTIMATED_Contact + 2;
    int idx_ESTIMATED_Hard_Contact  = idx_ESTIMATED_Slip + 2;
    int idx_ESTIMATED_rpy           = idx_ESTIMATED_Hard_Contact + 2;
    int idx_end                     = idx_ESTIMATED_rpy + 3;


    //Parameters
    bool textfile_flag = false;
    DigitCSVData csvData;

    bool slip_rejection_mode = false;
    double slip_threshold=0.0;

    bool variable_contact_cov_mode = false;
    double cov_amplifier = 2;


    // //not used
    // int Max_Iteration = 100;
    // double Optimization_Epsilon = 1e-3;


private:


};


#endif //SRC_InEKF_estimator_HPP
