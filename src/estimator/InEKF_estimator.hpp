
#ifndef SRC_InEKF_estimator_HPP
#define SRC_InEKF_estimator_HPP

// Libraries
#include <eigen3/Eigen/Dense>
#include "../InEKF/InEKF.h"
#include "../utility/BasicFunctions_Estimator.hpp"
#include "../utility/getParameters.hpp"

// ----------------------------
// Include robot model
#include "../utility/RobotModel.hpp"

// Include CSV data parser
#include "../utility/csvParser.hpp"
// ----------------------------

// Include ContactNN for neural network-based contact detection
#include "../contactDetection/ContactNN.hpp"
#include <deque>

using namespace Eigen;
using namespace std;

class InEKFilter {

public:
    // Robot State Type Definition - All variables needed for state representation
    typedef struct _ROBOT_STATES_ {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Matrix3d Rotation;
        Eigen::Vector3d Velocity;
        Eigen::Vector3d Angular_Velocity;
        Eigen::Vector3d Position;

        // Eigen::VectorXd d;        // Dynamic size, set at runtime based on num_legs
        Eigen::VectorXd d_v;      // Dynamic size, set at runtime based on num_legs
        Eigen::VectorXd dv_norm;  // Dynamic size, set at runtime based on num_legs

        Eigen::Vector3d Bias_Gyro;
        Eigen::Vector3d Bias_Acc;

        std::vector<bool> Contact;      // Dynamic size
        std::vector<bool> Hard_Contact;  // Dynamic size
        std::vector<bool> Slip;          // Dynamic size

    }ROBOT_STATES;


    // Constructor - automatically initializes from YAML config
    InEKFilter() = default;  // Default constructor (no initialization)
    InEKFilter(const std::string& initialization_config_path, const std::string& robot_config_path) {
        load_robot_config(robot_config_path);  // Load robot config FIRST
        InitializeFromYAML(initialization_config_path);  // Then initialize (loads contact_model_path and sets contact_nn_enabled)
        if (contact_nn_enabled) {
            load_contact_model(contact_model_path);  // Load ONNX contact estimation model only if enabled
        }
    }
    
    //functions
    void load_robot_config(const std::string& config_path) { 
        robot.load_config(config_path); 
        compute_indices(); // Compute indices after robot config is loaded
    }
    
    // Load ONNX contact estimation model (delegates to ContactNN)
    void load_contact_model(const std::string& model_path = "../src/estimator/bipedal_contact_best_val_leg_acc.onnx");
    
    void LoadCSVData(const std::string& csv_filename);

    void Initialize(double _dt, Eigen::Vector<double,12> cov_val_setting, Eigen::Vector<double, 16> initial_condition, int sliding_window_size = 2); // default sliding window size = 2 (filter)
    
    void InitializeFromYAML(const std::string& config_path); // Initialize from YAML configuration file

    void new_measurement(const Eigen::VectorXd &imu_i, const Eigen::MatrixXd &motor_data_i, const std::vector<bool> &Contact_i);
    void new_measurement(const Eigen::VectorXd &imu_i, const Eigen::MatrixXd &motor_data_i);  // Contact detection from torques
    void new_measurement();  // No-argument version for CSV mode

    std::vector<bool> contact_detection_logic(const std::vector<double> &motor_torques_i);  // Helper for contact detection from torques
    void Propagate_and_Correct();
    std::vector<double> Variable_Contact_Cov(int time);

    // void send_states(ROBOT_STATES &state_);
    void sliding_window();

    void Onestep(const Eigen::VectorXd &imu_i, const Eigen::MatrixXd &motor_data_i, const std::vector<bool> &Contact_i);
    void Onestep(const Eigen::VectorXd &imu_i, const Eigen::MatrixXd &motor_data_i, const Eigen::VectorXd &motor_torques_i, const Eigen::VectorXd &motor_pos_cmd_i, const Eigen::VectorXd &Cmd_Velocity_i);  // Contact detection from torques
    void Onestep();  // No-argument version for CSV mode
    // void Onestep(const Eigen::VectorXd &imu_i, const Eigen::MatrixXd &motor_data_i, const Eigen::VectorXd &motor_torques_i);  // Contact detection from torques

    // Static utility function - can be called without creating an estimator object
    static void getParameters(const std::string& urdf_path, const std::string& robot_name);

    int start_index = 0; // Starting row index in CSV data (skip initial bad data)

    //Necessary classes & variables
    RobotModel robot; // This is where robot model is defined
    
    // Neural network contact estimator
    std::unique_ptr<ContactNN> contact_nn;
    std::string contact_model_path;  // Path to the contact model loaded from YAML
    bool contact_nn_enabled = false;  // Flag to enable/disable NN contact detection

    int frame_count=0;
    int time_count=0;

    ROBOT_STATES robot_state;  // Current estimated state populated by send_states()

    double dt;
    Vector3d gravity{0, 0, -9.806659};


    //Inner variable
    inekf::InEKF filter;

    Eigen::Vector<double,6> imu_measurement;
    Eigen::Vector<double,6> imu_measurement_prev;
    
    // Estimated State Values Window Buffer (dynamically allocated based on SLIDING_WINDOW_SIZE)
    std::vector<Vector3d>                   Position_s;
    std::vector<Matrix3d>                   Rotation_s;
    std::vector<Vector3d>                   Velocity_s;
    std::vector<Eigen::VectorXd>            d_v;
    std::vector<Eigen::VectorXd>            dv_norm;     // Scalar velocity norms per leg

    std::vector<Vector3d>                   Bias_Acc_s;
    std::vector<Vector3d>                   Bias_Gyro_s;

    // True Values and measurement Buffer (dynamically allocated based on SLIDING_WINDOW_SIZE)
    std::vector<std::vector<bool>>     CONTACT_t;
    std::vector<std::vector<bool>>     HARD_CONTACT_t;
    std::vector<std::vector<bool>>     SLIP_t;

    std::vector<Eigen::Vector3d>    IMU_Gyro;
    std::vector<Eigen::Vector3d>    IMU_Acc;
    std::vector<Eigen::VectorXd>   ENCODER;      // Dynamic size, set after robot config loaded
    std::vector<Eigen::VectorXd>   ENCODERDOT;   // Dynamic size, set after robot config loaded
    std::vector<Eigen::VectorXd>   MOTOR_TORQUES;     // Raw motor torques for NN features
    std::vector<Eigen::VectorXd>   MOTOR_POS_CMD; // Commanded motor positions for logging and potential use in contact detection
    std::vector<Eigen::VectorXd>   CMD_V;            // Commanded velocities (if available in CSV data)
    std::vector<Eigen::VectorXd>   torqueSumSquared;  // Sliding window buffer for torque sum squared
    //SAVE PARAMETER
    std::string estimator_info;
    std::string file_info;
    std::string initial_info;
    std::string cov_info;

    const static int SAVEMAXDATA = 100;
    const static int SAVEMAXCNT = 140000;
    std::vector<std::vector<double>> SAVE_MATRIX;  // Changed to vector to avoid stack overflow
    int save_index = 0;

    // File save indices - computed after robot config is loaded
    int idx_TRUE_Rotation;
    int idx_TRUE_Velocity;
    int idx_TRUE_Position;
    int idx_TRUE_Omega;
    int idx_TRUE_dv;
    int idx_TRUE_dv_norm;
    int idx_TRUE_Bias_Gyro;
    int idx_TRUE_Bias_Acc;
    int idx_TRUE_Contact;
    int idx_TRUE_Slip;
    int idx_TRUE_Hard_Contact;
    int idx_TRUE_rpy;

    int idx_ESTIMATED_Rotation;
    int idx_ESTIMATED_Velocity;
    int idx_ESTIMATED_Position;
    int idx_ESTIMATED_Omega;
    int idx_ESTIMATED_dv;
    int idx_ESTIMATED_dv_norm;
    int idx_ESTIMATED_Bias_Gyro;
    int idx_ESTIMATED_Bias_Acc;
    int idx_ESTIMATED_Contact;
    int idx_ESTIMATED_Slip;
    int idx_ESTIMATED_Hard_Contact;
    int idx_ESTIMATED_rpy;
    int idx_end;

    //Parameters
    bool csv_mode = false;
    DataStruct CSVData; // Robot data from CSV parser
    int time_length = 0;  // Total number of data points to process (from YAML)

    bool slip_rejection_mode = false;
    double slip_threshold=0.0;

    bool variable_contact_cov_mode = false;
    double cov_amplifier = 2;

    std::string saved_results_folder = "../results";

    // dv history for CSV logging: [time_step][leg] = dv_norm
    std::vector<std::vector<double>> dv_history;

    // Contact detection accuracy tracking
    std::vector<bool> GroundTruthContact;  // Ground truth contact state
    std::vector<int> contact_match_count;   // Counts per leg
    std::vector<int> contact_mismatch_count; // Counts per leg
    int total_comparisons = 0;  // Total number of timesteps compared

private:
    // Helper function to compute indices after robot config is loaded
    void compute_indices() {
        idx_TRUE_Rotation = 0;
        idx_TRUE_Velocity = idx_TRUE_Rotation + 9;
        idx_TRUE_Position = idx_TRUE_Velocity + 3;
        idx_TRUE_Omega = idx_TRUE_Position + 3;
        idx_TRUE_dv = idx_TRUE_Omega + 3;
        idx_TRUE_dv_norm = idx_TRUE_dv + robot.num_legs * 3;
        idx_TRUE_Bias_Gyro = idx_TRUE_dv_norm + robot.num_legs;
        idx_TRUE_Bias_Acc = idx_TRUE_Bias_Gyro + 3;
        idx_TRUE_Contact = idx_TRUE_Bias_Acc + 3;
        idx_TRUE_Slip = idx_TRUE_Contact + robot.num_legs;
        idx_TRUE_Hard_Contact = idx_TRUE_Slip + robot.num_legs;
        idx_TRUE_rpy = idx_TRUE_Hard_Contact + robot.num_legs;

        idx_ESTIMATED_Rotation = idx_TRUE_rpy + 3;
        idx_ESTIMATED_Velocity = idx_ESTIMATED_Rotation + 9;
        idx_ESTIMATED_Position = idx_ESTIMATED_Velocity + 3;
        idx_ESTIMATED_Omega = idx_ESTIMATED_Position + 3;
        idx_ESTIMATED_dv = idx_ESTIMATED_Omega + 3;
        idx_ESTIMATED_dv_norm = idx_ESTIMATED_dv + robot.num_legs * 3;
        idx_ESTIMATED_Bias_Gyro = idx_ESTIMATED_dv_norm + robot.num_legs;
        idx_ESTIMATED_Bias_Acc = idx_ESTIMATED_Bias_Gyro + 3;
        idx_ESTIMATED_Contact = idx_ESTIMATED_Bias_Acc + 3;
        idx_ESTIMATED_Slip = idx_ESTIMATED_Contact + robot.num_legs;
        idx_ESTIMATED_Hard_Contact = idx_ESTIMATED_Slip + robot.num_legs;
        idx_ESTIMATED_rpy = idx_ESTIMATED_Hard_Contact + robot.num_legs;
        idx_end = idx_ESTIMATED_rpy + 3;
    }

};
#endif //SRC_InEKF_estimator_HPP