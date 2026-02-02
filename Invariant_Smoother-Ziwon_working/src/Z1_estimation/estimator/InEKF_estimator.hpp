
#ifndef SRC_InEKF_estimator_HPP
#define SRC_InEKF_estimator_HPP

// Libraries
#include <eigen3/Eigen/Dense>
#include "../InEKF/InEKF.h"
#include "../utility/BasicFunctions_Estimator.hpp"

// ----------------------------
// Include robot model
#include "../utility/RobotModel.hpp"

// Include CSV data types - HAVE TO CHANGE TO A UNIVERSAL CSV FILE READER LATER
#include "../utility/Digit/DigitCSVData.hpp"

// ----------------------------

using namespace Eigen;
using namespace std;

// Robot State Type Definition - All variables needed for state representation
typedef struct _ROBOT_STATES_ // I dont even need this maybe..send_States is not doing anything
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Matrix3d Rotation;
    Eigen::Vector3d Velocity;
    Eigen::Vector3d Position;

    Eigen::VectorXd d;        // Dynamic size, set at runtime based on num_legs
    Eigen::VectorXd d_v;      // Dynamic size, set at runtime based on num_legs

    Eigen::Vector3d Bias_Gyro;
    Eigen::Vector3d Bias_Acc;

    std::vector<bool> Contact;      // Dynamic size
    std::vector<bool> Hard_Contact;  // Dynamic size
    std::vector<bool> Slip;          // Dynamic size

    int contact_leg_num; // delete everything below this later

    int state_size;
    int state_idx;
    int para_size;
    int para_idx;

}ROBOT_STATES;

class InEKFilter {


public:
    //functions
    void load_robot_config(const std::string& config_path) { 
        robot.load_config(config_path); 
        compute_indices(); // Compute indices after robot config is loaded
    }
    void LoadCSVData(const std::string& csv_filename);
    // int SLIDING_WINDOW_SIZE = 2; // Runtime configurable (set in Initialize)
    void Initialize(double _dt, Eigen::Vector<double,12> cov_val_setting, Eigen::Vector<double, 16> initial_condition, int sliding_window_size = 2); // default sliding window size = 2 (filter)

    void new_measurement(Eigen::Vector<double,30> &Sensor_i, std::vector<bool> &Contact_i);

    void Propagate_and_Correct();
    std::vector<double> Variable_Contact_Cov(int time);

    void send_states(ROBOT_STATES &state_);
    void sliding_window();

    void SAVE_onestep(int save_index);

    void Onestep(Eigen::Vector<double,30> &Sensor_i, std::vector<bool> &Contact_i,ROBOT_STATES &state_);

    void writeSavedData(std::string cov_info);


    int CSV_Skip_Data = 0; // default (change in main.cpp if you have trash data)


    //Necessary classes & variables
    RobotModel robot; // This is where robot model is defined

    int frame_count=0;
    int time_count=0;


    double dt;
    Vector3d gravity{0, 0, -9.806659};



    //Call_FILE PARAMETER
    // const static int MAX_FILE_COUNT =140000;
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
    std::vector<Eigen::VectorXd>            d_v;

    std::vector<Vector3d>                   Bias_Acc_s;
    std::vector<Vector3d>                   Bias_Gyro_s;

    // True Values and measurement Buffer (dynamically allocated based on SLIDING_WINDOW_SIZE)
    std::vector<std::vector<bool>>     HARD_CONTACT_t;
    std::vector<std::vector<bool>>     CONTACT_t;
    std::vector<std::vector<bool>>     SLIP_t;

    std::vector<Eigen::Vector3d>            IMU_Gyro;
    std::vector<Eigen::Vector3d>            IMU_Acc;
    std::vector<Eigen::VectorXd>   ENCODER;      // Dynamic size, set after robot config loaded
    std::vector<Eigen::VectorXd>   ENCODERDOT;   // Dynamic size, set after robot config loaded

    //SAVE PARAMETER
    std::string estimator_info;
    std::string file_info;
    std::string initial_info;

    const static int SAVEMAXDATA = 100;
    const static int SAVEMAXCNT = 140000;
    double SAVE_ROW [SAVEMAXDATA][SAVEMAXCNT];
    int save_index = 0;

    // File save indices - computed after robot config is loaded
    int idx_TRUE_Rotation;
    int idx_TRUE_Velocity;
    int idx_TRUE_Position;
    int idx_TRUE_dv;
    int idx_TRUE_Bias_Gyro;
    int idx_TRUE_Bias_Acc;
    int idx_TRUE_Contact;
    int idx_TRUE_Slip;
    int idx_TRUE_Hard_Contact;
    int idx_TRUE_rpy;

    int idx_ESTIMATED_Rotation;
    int idx_ESTIMATED_Velocity;
    int idx_ESTIMATED_Position;
    int idx_ESTIMATED_dv;
    int idx_ESTIMATED_Bias_Gyro;
    int idx_ESTIMATED_Bias_Acc;
    int idx_ESTIMATED_Contact;
    int idx_ESTIMATED_Slip;
    int idx_ESTIMATED_Hard_Contact;
    int idx_ESTIMATED_rpy;
    int idx_end;

    //Parameters
    bool textfile_flag = false;
    DigitCSVData csvData; // CSV data type - using Digit format for now

    bool slip_rejection_mode = false;
    double slip_threshold=0.0;

    bool variable_contact_cov_mode = false;
    double cov_amplifier = 2;


private:
    // Helper function to compute indices after robot config is loaded
    void compute_indices() {
        idx_TRUE_Rotation = 0;
        idx_TRUE_Velocity = idx_TRUE_Rotation + 9;
        idx_TRUE_Position = idx_TRUE_Velocity + 3;
        idx_TRUE_dv = idx_TRUE_Position + 3;
        idx_TRUE_Bias_Gyro = idx_TRUE_dv + robot.num_legs * 3;
        idx_TRUE_Bias_Acc = idx_TRUE_Bias_Gyro + 3;
        idx_TRUE_Contact = idx_TRUE_Bias_Acc + 3;
        idx_TRUE_Slip = idx_TRUE_Contact + robot.num_legs;
        idx_TRUE_Hard_Contact = idx_TRUE_Slip + robot.num_legs;
        idx_TRUE_rpy = idx_TRUE_Hard_Contact + robot.num_legs;

        idx_ESTIMATED_Rotation = idx_TRUE_rpy + 3;
        idx_ESTIMATED_Velocity = idx_ESTIMATED_Rotation + 9;
        idx_ESTIMATED_Position = idx_ESTIMATED_Velocity + 3;
        idx_ESTIMATED_dv = idx_ESTIMATED_Position + 3;
        idx_ESTIMATED_Bias_Gyro = idx_ESTIMATED_dv + robot.num_legs * 3;
        idx_ESTIMATED_Bias_Acc = idx_ESTIMATED_Bias_Gyro + 3;
        idx_ESTIMATED_Contact = idx_ESTIMATED_Bias_Acc + 3;
        idx_ESTIMATED_Slip = idx_ESTIMATED_Contact + robot.num_legs;
        idx_ESTIMATED_Hard_Contact = idx_ESTIMATED_Slip + robot.num_legs;
        idx_ESTIMATED_rpy = idx_ESTIMATED_Hard_Contact + robot.num_legs;
        idx_end = idx_ESTIMATED_rpy + 3;
    }

};
#endif //SRC_InEKF_estimator_HPP