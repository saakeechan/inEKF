//
// Created by Junny on  2020-06-29.
// Last Update on       2020-06-30
//

#ifndef SRC_PSEUDO_INVARIANT_ESTIMATOR_HPP
#define SRC_PSEUDO_INVARIANT_ESTIMATOR_HPP

// Libraries
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <fstream>

// Necessaries
#include "Z1_Parameters.hpp"
#include "RobotState_Smoother.hpp"
#include "../utility/BasicFunctions_Estimator.hpp"
#include "../utility/Robot_Model_mini_cheetah.hpp"
#include "../utility/tic_toc.h"

// Factors
#include "../factor/Factors.hpp"



using namespace Eigen;
using namespace std;

using Covariance_parameter=Eigen::Matrix<double,11,1>;
using Sensor_block=Eigen::Matrix<double,30,1>;
using Contact_block=Eigen::Matrix<bool,4,1>;




class Pseudo_Invariant_estimator {

public:


    int NUM_OF_TRASH_DATA = 1;

    const static int num_z_imu=6;
    const static int num_z_encoder=12;
    const static int num_z_encoderdot=12;
    const static int num_z = num_z_imu + num_z_encoder + num_z_encoderdot;

    Eigen::Matrix<double,15*(WINDOW_SIZE+1),1>                      perturbation;
    Eigen::Matrix<double,15*(WINDOW_SIZE+1),1>                      delta_Zeta_Xi;

    Eigen::Matrix<double,num_z*(WINDOW_SIZE + 1),1>             Estimation_Z;

    Eigen::Matrix<double,15,15>         Marginalized_H;
    Eigen::Matrix<double,15,1>                      Marginalized_b;

    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>         Hessian_S;


    //functions
    void Call_File(std::string file_name);
    //void Initialize(double _dt, Eigen::Matrix<double,23,1> &COV_IC_setting);

    void Initialize(double _dt, Eigen::Matrix<double,12,1> &cov_val_setting, Eigen::Matrix<double,16,1> &initial_condition);

    void new_measurement(Eigen::Matrix<double,30,1> &Sensor_i, Eigen::Matrix<bool,4,1> &Contact_i);
    void Optimization_Solve();
    void retract_manifold(int start_frame);
    void update_dv(int start_frame);
    void sliding_window();

    void send_states(ROBOT_STATES &state_);

    void SAVE_onestep_Z1(int cnt);
    void do_SAVE_Z1_all(std::string cov_info);

    void Onestep(Eigen::Matrix<double,30,1> &Sensor_i, Eigen::Matrix<bool,4,1> &Contact_i,ROBOT_STATES &state_);

    std::vector<double> Variable_Contact_Cov(int time);




    //Necessary classes & variables
    Robot_Model_mini_cheetah robot;
    Factors factors;

    bool sliding_window_flag=false;
    bool marginalization_flag = false;

    int frame_count=0;
    int time_count=0;

    double dt=robot.dt;
    Vector3d gravity;

    // Estimated State Values Window Buffer
    ROBOT_STATES_PSEUDO RS [WINDOW_SIZE+1];
    ROBOT_STATES_PSEUDO RS_Pri;


    factor_info_pseudo fac_info [WINDOW_SIZE+1];






    //Call_FILE PARAMETER
    int gt_sd = 0;
    int row_index = 0; // row index
    int column_index = 0; // column index
    const static int MAX_FILE_COUNT =40000;
    int max_time = MAX_FILE_COUNT;

    double SensorData       [MAX_FILE_COUNT][30];
    double GroundTruth      [MAX_FILE_COUNT][27];



    //SAVE PARAMETER
    const static int SAVEMAX = 103;
    const static int SAVEMAXCNT = MAX_FILE_COUNT;
    double SAVE_Z1 [SAVEMAX][SAVEMAXCNT];
    int SAVE_cnt = 0;

    int iteration_number = 0;
    int total_backppgn_number = 0;
    double time_per_step;

    std::string estimator_info;
    std::string file_info;
    std::string initial_info;
    std::string time_size;
    std::string est_size;

    int idx_TRUE_Rotation           = 0;//0
    int idx_TRUE_Velocity           = idx_TRUE_Rotation + 9;//9
    int idx_TRUE_Position           = idx_TRUE_Velocity + 3;//12
    int idx_TRUE_dv                  = idx_TRUE_Position + 3;//15
    int idx_TRUE_Bias_Gyro          = idx_TRUE_dv + 12;//27
    int idx_TRUE_Bias_Acc           = idx_TRUE_Bias_Gyro + 3;//30
    int idx_TRUE_Contact            = idx_TRUE_Bias_Acc + 3;//33
    int idx_TRUE_Slip               = idx_TRUE_Contact + 4;//37
    int idx_TRUE_Hard_Contact       = idx_TRUE_Slip+ 4;//41
    int idx_TRUE_rpy                = idx_TRUE_Hard_Contact + 4;//45

    int idx_ESTIMATED_Rotation      = idx_TRUE_rpy + 3;//48
    int idx_ESTIMATED_Velocity      = idx_ESTIMATED_Rotation + 9;//57
    int idx_ESTIMATED_Position      = idx_ESTIMATED_Velocity + 3;//60
    int idx_ESTIMATED_dv            = idx_ESTIMATED_Position + 3;//63
    int idx_ESTIMATED_Bias_Gyro     = idx_ESTIMATED_dv + 12;//75
    int idx_ESTIMATED_Bias_Acc      = idx_ESTIMATED_Bias_Gyro + 3;//78
    int idx_ESTIMATED_Contact       = idx_ESTIMATED_Bias_Acc + 3;//81
    int idx_ESTIMATED_Slip          = idx_ESTIMATED_Contact + 4;//85
    int idx_ESTIMATED_Hard_Contact  = idx_ESTIMATED_Slip + 4;//89
    int idx_ESTIMATED_rpy           = idx_ESTIMATED_Hard_Contact + 4;//93

    int idx_iteration_No            = idx_ESTIMATED_rpy + 3;//96
    int idx_backppgn_No             = idx_iteration_No + 1;//97
    int idx_time_per_step           = idx_backppgn_No +1;//98
    int idx_cost                    = idx_time_per_step +1;//99
    int idx_end                     = idx_cost + 4;//103

    Eigen::Matrix<double, 4,1> cost_array;
    Eigen::Matrix<double,100,1> debug_array;



    //Parameters Setting
    bool textfile_flag=false;

    bool Retract_All_flag = false;

    int Max_Iteration = 100;
    double Optimization_Epsilon = 1e-3;

    int Max_backpropagate_num = 3;
    //To turn off backpropagation, set num=0
    double backppgn_rate = 0.5;

    bool slip_rejection_mode=false;
    double slip_threshold = 0;

    bool variable_contact_cov_mode = false;
    double cov_amplifier = 2;




private:



};


#endif //SRC_PSEUDO_INVARIANT_ESTIMATOR_HPP
