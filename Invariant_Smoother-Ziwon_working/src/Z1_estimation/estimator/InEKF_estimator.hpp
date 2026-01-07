//
// Created by Junny on  2020-06-29.
// Last Update on       2020-06-30
//

#ifndef SRC_InEKF_estimator_HPP
#define SRC_InEKF_estimator_HPP

// Libraries
#include <eigen3/Eigen/Dense>
#include "../InEKF/InEKF.h"
#include "../utility/BasicFunctions_Estimator.hpp"

// ----------------------------
// Include robot configuration (automatically includes correct robot model)
// To use a different robot, edit ../utility/RobotConfig.hpp
#include "../utility/RobotConfig.hpp"

// ----------------------------

using namespace Eigen;
using namespace std;

// Robot State Type Definition
typedef struct _ROBOT_STATES_
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix3d Rotation;
    Eigen::Vector3d Velocity;
    Eigen::Vector3d Position;

    Eigen::Vector<double, Robot_Model::NUM_LEGS*3> d;
    Eigen::Vector<double, Robot_Model::NUM_LEGS*3> d_v;

    Eigen::Vector3d Bias_Gyro;
    Eigen::Vector3d Bias_Acc;

    Eigen::Vector<bool, Robot_Model::NUM_LEGS> Contact;
    Eigen::Vector<bool, Robot_Model::NUM_LEGS> Hard_Contact;
    Eigen::Vector<bool, Robot_Model::NUM_LEGS> Slip;

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
    void Initialize(double _dt, Eigen::Vector<double,12> cov_val_setting, Eigen::Vector<double, 16> initial_condition, int sliding_window_size = 2);

    void new_measurement(Eigen::Vector<double,30> &Sensor_i, Eigen::Vector<bool,Robot_Model::NUM_LEGS> &Contact_i);

    void Propagate_Correct();
    std::vector<double> Variable_Contact_Cov(int time);

    void send_states(ROBOT_STATES &state_);
    void sliding_window();

    void SAVE_onestep_Z1(int cnt);

    void Onestep(Eigen::Vector<double,30> &Sensor_i, Eigen::Vector<bool,Robot_Model::NUM_LEGS> &Contact_i,ROBOT_STATES &state_);

    void do_SAVE_Z1_all(std::string cov_info);


    int NUM_OF_TRASH_DATA = 0; // default (change in main.cpp if you have trash data)


    //Necessary classes & variables
    Robot_Model robot; // This is where robot model is defined

    bool sliding_window_flag=false;
    int frame_count=0;
    int time_count=0;
    int SLIDING_WINDOW_SIZE = 2; // Runtime configurable (set in Initialize)

    double dt;
    Vector3d gravity;



    //Call_FILE PARAMETER
    int gt_sd = 0;
    const static int MAX_FILE_COUNT =140000;
    // double SensorData       [MAX_FILE_COUNT][30];
    // double GroundTruth      [MAX_FILE_COUNT][27];


    //Inner variable
    inekf::InEKF filter;

    Eigen::Vector<double,6> imu_measurement;
    Eigen::Vector<double,6> imu_measurement_prev;
    
    // Estimated State Values Window Buffer (dynamically allocated based on SLIDING_WINDOW_SIZE)
    std::vector<Vector3d>                   Position_s;
    std::vector<Matrix3d>                   Rotation_s;
    std::vector<Vector3d>                   Velocity_s;
    std::vector<Eigen::Vector<double,Robot_Model::NUM_LEGS*3>>    d_v;

    std::vector<Vector3d>                   Bias_Acc_s;
    std::vector<Vector3d>                   Bias_Gyro_s;
    std::vector<Eigen::Vector<double, 6>>   Bias_s;

    // True Values and measurement Buffer (dynamically allocated based on SLIDING_WINDOW_SIZE)
    std::vector<Eigen::Vector<bool, Robot_Model::NUM_LEGS>>     HARD_CONTACT_t;
    std::vector<Eigen::Vector<bool, Robot_Model::NUM_LEGS>>     CONTACT_t;
    std::vector<Eigen::Vector<bool, Robot_Model::NUM_LEGS>>     SLIP_t;

    std::vector<Eigen::Vector3d>            IMU_Gyro;
    std::vector<Eigen::Vector3d>            IMU_Acc;
    std::vector<Eigen::Vector<double,Robot_Model::ENCODER_SIZE>>   ENCODER;
    std::vector<Eigen::Vector<double,Robot_Model::ENCODERDOT_SIZE>>   ENCODERDOT;

    //SAVE PARAMETER
    std::string estimator_info;
    std::string file_info;
    std::string initial_info;

    const static int SAVEMAX = 100;
    const static int SAVEMAXCNT = MAX_FILE_COUNT;
    double SAVE_Z1 [SAVEMAX][SAVEMAXCNT];
    int SAVE_cnt = 0;

    int idx_TRUE_Rotation           = 0;
    int idx_TRUE_Velocity           = idx_TRUE_Rotation + 9;
    int idx_TRUE_Position           = idx_TRUE_Velocity + 3;
    int idx_TRUE_dv                  = idx_TRUE_Position + 3;
    int idx_TRUE_Bias_Gyro          = idx_TRUE_dv + Robot_Model::NUM_LEGS*3;
    int idx_TRUE_Bias_Acc           = idx_TRUE_Bias_Gyro + 3;
    int idx_TRUE_Contact            = idx_TRUE_Bias_Acc + 3;
    int idx_TRUE_Slip               = idx_TRUE_Contact + Robot_Model::NUM_LEGS;
    int idx_TRUE_Hard_Contact       = idx_TRUE_Slip+ Robot_Model::NUM_LEGS;
    int idx_TRUE_rpy                = idx_TRUE_Hard_Contact + Robot_Model::NUM_LEGS;

    int idx_ESTIMATED_Rotation      = idx_TRUE_rpy + 3;
    int idx_ESTIMATED_Velocity      = idx_ESTIMATED_Rotation + 9;
    int idx_ESTIMATED_Position      = idx_ESTIMATED_Velocity + 3;
    int idx_ESTIMATED_dv             = idx_ESTIMATED_Position + 3;
    int idx_ESTIMATED_Bias_Gyro     = idx_ESTIMATED_dv + Robot_Model::NUM_LEGS*3;
    int idx_ESTIMATED_Bias_Acc      = idx_ESTIMATED_Bias_Gyro + 3;
    int idx_ESTIMATED_Contact       = idx_ESTIMATED_Bias_Acc + 3;
    int idx_ESTIMATED_Slip          = idx_ESTIMATED_Contact + Robot_Model::NUM_LEGS;
    int idx_ESTIMATED_Hard_Contact  = idx_ESTIMATED_Slip + Robot_Model::NUM_LEGS;
    int idx_ESTIMATED_rpy           = idx_ESTIMATED_Hard_Contact + Robot_Model::NUM_LEGS;
    int idx_end                     = idx_ESTIMATED_rpy + 3;

    //Parameters
    bool textfile_flag = false;
    RobotCSVData csvData; // Robot-specific CSV data type

    bool slip_rejection_mode = false;
    double slip_threshold=0.0;

    bool variable_contact_cov_mode = false;
    double cov_amplifier = 2;


private:

};


#endif //SRC_InEKF_estimator_HPP

// variables for number of legs, sliding window size for potential smoother, joint indices for different robots, 