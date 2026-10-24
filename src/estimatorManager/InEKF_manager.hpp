
#ifndef SRC_InEKF_manager_HPP
#define SRC_InEKF_manager_HPP

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
#include <tuple>
#include <optional>
#include <stdexcept>

using namespace Eigen;
using namespace std;

struct ContactState {
    std::vector<bool> contact;
    std::vector<bool> hard_contact;
    std::vector<bool> slip;
    Eigen::VectorXd d_v;
    Eigen::VectorXd dv_norm;

    void reset(int num_legs, double velocity_value) {
        contact.assign(num_legs, false);
        hard_contact.assign(num_legs, false);
        slip.assign(num_legs, false);
        d_v.setConstant(3 * num_legs, velocity_value);
        dv_norm.setConstant(num_legs, velocity_value);
    }
};

struct KinematicsMeasurement {
    std::vector<bool> contacts;
    Eigen::MatrixXd motor_data;
};

struct BodyVelocityMeasurement {
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero(); // IMU origin in IMU axes.
    Eigen::Vector3d variance = Eigen::Vector3d::Zero(); // Positive diagonal variances.
};

struct StepMeasurements {
    std::optional<KinematicsMeasurement> kinematics;
    std::optional<BodyVelocityMeasurement> body_velocity;
};

class RobotManager {
public:
    // Robot State Type Definition - All variables needed for state representation
    typedef struct _ManagerState_ {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Matrix3d Rotation;
        Eigen::Vector3d Velocity;
        Eigen::Vector3d Angular_Velocity; // Selected frame angular velocity in world axes.
        Eigen::Vector3d Position;
        Eigen::MatrixXd Covariance;  // Raw IMU filter error-state covariance; not transformed to stateFrame_id.

        // Eigen::VectorXd d;        // Dynamic size, set at runtime based on num_legs
        Eigen::VectorXd d_v;      // Dynamic size, set at runtime based on num_legs
        Eigen::VectorXd dv_norm;  // Dynamic size, set at runtime based on num_legs

        Eigen::Vector3d Bias_Gyro;
        Eigen::Vector3d Bias_Acc;

        std::vector<bool> Contact;      // Dynamic size
        std::vector<bool> Hard_Contact;  // Dynamic size
        std::vector<bool> Slip;          // Dynamic size

    }ManagerState;

    RobotModel model;
    ManagerState state;  // Latest output populated by send_states().
};

class INEKFilter_manager {

public:
    // Constructor - automatically initializes from YAML config
    INEKFilter_manager() = default;  // Default constructor (no initialization)
    INEKFilter_manager(const std::string& initialization_config_path_arg, const std::string& robot_config_path_arg) {
        load_robot_config(robot_config_path_arg);  // Load robot config FIRST
        InitializeFromYAML(initialization_config_path_arg);  // Then initialize (loads contact_model_path and sets contact_nn_enabled)
        if (!csv_mode && contact_nn_enabled) {
            load_contact_model(contact_model_path);  // Load ONNX contact estimation model only if enabled
        }
    }
    
    //functions
    void load_robot_config(const std::string& config_path_arg) {
        if (!robot.model.load_config(config_path_arg)) {
            throw std::runtime_error("Failed to load robot configuration: " + config_path_arg);
        }
        compute_indices(); // Compute indices after robot config is loaded
    }
    
    // Load ONNX contact estimation model (delegates to ContactNN)
    void load_contact_model(const std::string& model_path_arg = "../src/estimatorManager/bipedal_contact_best_val_leg_acc.onnx");
    
    void LoadCSVData(const std::string& csv_filename_arg);

    void Initialize(double dt_arg, Eigen::Vector<double,12> cov_val_setting_arg, Eigen::Vector<double, 16> initial_condition_arg, int sliding_window_size_arg = 2); // default sliding window size = 2 (filter)
    
    void InitializeFromYAML(const std::string& config_path_arg); // Initialize from YAML configuration file

    void new_measurement(const Eigen::VectorXd &imu_arg, const Eigen::MatrixXd &motor_data_arg, const std::vector<bool> &contacts_arg);
    void new_measurement(const Eigen::VectorXd &imu_arg);  // IMU-only mode

    std::vector<bool> contact_detection_logic(const std::vector<double> &motor_torques_arg);  // Helper for contact detection from torques
    void correct_kinematics();
    void update_contact_diagnostics();
    std::vector<double> Variable_Contact_Cov(int frame_index_arg);

    void send_states(RobotManager::ManagerState &state_arg);
    void store_state_history(inekf::RobotState& state_arg);
    void sliding_window();

    // Supply fresh IMU; propagation uses the previous sample in the window.
    void Onestep(const Eigen::VectorXd &fresh_imu_arg, const StepMeasurements &measurements_arg = {});

    // Independent live contact detectors; call the YAML-selected detector before Onestep.
    // Returns contacts and per-leg norms of NN-predicted world foot velocities.
    std::tuple<std::vector<bool>, Eigen::VectorXd> run_contact_inference(const Eigen::VectorXd &imu_arg, const Eigen::MatrixXd &motor_data_arg,
                                          const Eigen::VectorXd &motor_torques_arg, const Eigen::VectorXd &motor_pos_cmd_arg,
                                          const Eigen::VectorXd &cmd_velocity_arg);
    // Pass one latest frame of torques and joint velocities, not their sliding windows.
    std::vector<bool> run_contact_detection(const Eigen::VectorXd &motor_torques_arg,
                                          const Eigen::VectorXd &joint_velocities_arg);
    Eigen::VectorXd ReadCSVIMU(int row_arg) const;
    std::tuple<Eigen::VectorXd, std::vector<bool>, Eigen::MatrixXd> ReadCSVInputs(int row_arg) const;
    void RunCSVStep();

    // Static utility function - can be called without creating an estimator object
    static void getParameters(const std::string& urdf_path_arg, const std::string& robot_name_arg);

    int start_index = 0; // Starting row index in CSV data (skip initial bad data)

    //Necessary classes & variables
    RobotManager robot; // Kinematic model and latest output state.
    
    // Neural network contact estimator
    std::unique_ptr<ContactNN> contact_nn;
    std::string contact_model_path;  // Path to the contact model loaded from YAML
    bool contact_nn_enabled = false;  // Flag to enable/disable NN contact detection

    int sliding_window_size = 2; // Configured by YAML and applied by Initialize.
    int frame_count=0; // Fresh slot is N-1; latest completed frame is N-2.
    int time_count=0;


    double dt;
    Vector3d gravity{0, 0, -9.806659};


    //Inner variable
    inekf::InEKF filter;

    Eigen::Vector<double,6> imu_measurement;
    bool imu_history_initialized = false;
    
    // Estimated State Values Window Buffer (dynamically allocated based on sliding_window_size)
    std::vector<Vector3d>                   Position_s;
    std::vector<Matrix3d>                   Rotation_s;
    std::vector<Vector3d>                   Velocity_s;

    std::vector<Vector3d>                   Bias_Acc_s;
    std::vector<Vector3d>                   Bias_Gyro_s;

    // True Values and measurement Buffer (dynamically allocated based on sliding_window_size)
    std::vector<ContactState> CONTACT_s;

    std::vector<Eigen::Vector3d>    IMU_Gyro;
    std::vector<Eigen::Vector3d>    IMU_Acc;
    std::vector<Eigen::VectorXd>   ENCODER;      // Dynamic size, set after robot config loaded
    std::vector<Eigen::VectorXd>   ENCODERDOT;   // Dynamic size, set after robot config loaded
    std::vector<Eigen::VectorXd>   MOTOR_TORQUES;     // Raw motor torques for NN features
    Eigen::VectorXd dv_norm_nn;  // Pending NN norms for the next step; empty when no inference ran.
    Eigen::VectorXd NN_foot_velocity_world;  // [x, y, z] world velocity for each leg
    Eigen::VectorXd NN_foot_velocity_variance_world;  // [vx, vy, vz] diagonal variance for each leg
    //SAVE PARAMETER
    std::string estimator_info;
    std::string file_info;
    std::string initial_info;
    std::string cov_info;

    const static int SAVEMAXDATA = 100;
    const static int SAVEMAXCNT = 140000;
    std::vector<std::vector<double>> SAVE_MATRIX;  // Changed to vector to avoid stack overflow
    int saved_count = 0;  // Buffered rows, including initialization; independent of time_count.

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
    int time_length = 0;  // Inclusive final CSV row index (end_count from YAML).

    bool slip_rejection_mode = false;
    double slip_threshold=0.0;

    // Effective estimator covariances: robot defaults overridden by initialization YAML.
    double cov_gyro_const = 0.0;
    double cov_acc_const = 0.0;
    double cov_slip_const = 0.0;
    double cov_contact_const = 0.0;
    double cov_enc_const = 0.0;
    double cov_bias_gyro_const = 0.0;
    double cov_bias_acc_const = 0.0;
    double cov_prior_orientation_const = 0.0;
    double cov_prior_velocity_const = 0.0;
    double cov_prior_position_const = 0.0;
    double cov_prior_bias_gyro_const = 0.0;
    double cov_prior_bias_acc_const = 0.0;

    bool variable_contact_cov_mode = false;
    double cov_amplifier = 2;

    bool save_results = true;
    std::string saved_results_folder = "../results";

    // Contact detection accuracy tracking
    std::vector<bool> GroundTruthContact;  // Ground truth contact state
    std::vector<int> contact_match_count;   // Counts per leg
    std::vector<int> contact_mismatch_count; // Counts per leg
    int total_comparisons = 0;  // Total number of timesteps compared

private:
    bool divergence_reported = false;
    std::string kinematics_mode = "pkm";

    // Helper function to compute indices after robot config is loaded
    void compute_indices() {
        idx_TRUE_Rotation = 0;
        idx_TRUE_Velocity = idx_TRUE_Rotation + 9;
        idx_TRUE_Position = idx_TRUE_Velocity + 3;
        idx_TRUE_Omega = idx_TRUE_Position + 3;
        idx_TRUE_dv = idx_TRUE_Omega + 3;
        idx_TRUE_dv_norm = idx_TRUE_dv + robot.model.num_legs * 3;
        idx_TRUE_Bias_Gyro = idx_TRUE_dv_norm + robot.model.num_legs;
        idx_TRUE_Bias_Acc = idx_TRUE_Bias_Gyro + 3;
        idx_TRUE_Contact = idx_TRUE_Bias_Acc + 3;
        idx_TRUE_rpy = idx_TRUE_Contact + robot.model.num_legs;

        idx_ESTIMATED_Rotation = idx_TRUE_rpy + 3;
        idx_ESTIMATED_Velocity = idx_ESTIMATED_Rotation + 9;
        idx_ESTIMATED_Position = idx_ESTIMATED_Velocity + 3;
        idx_ESTIMATED_Omega = idx_ESTIMATED_Position + 3;
        idx_ESTIMATED_dv = idx_ESTIMATED_Omega + 3;
        idx_ESTIMATED_dv_norm = idx_ESTIMATED_dv + robot.model.num_legs * 3;
        idx_ESTIMATED_Bias_Gyro = idx_ESTIMATED_dv_norm + robot.model.num_legs;
        idx_ESTIMATED_Bias_Acc = idx_ESTIMATED_Bias_Gyro + 3;
        idx_ESTIMATED_Contact = idx_ESTIMATED_Bias_Acc + 3;
        idx_ESTIMATED_Slip = idx_ESTIMATED_Contact + robot.model.num_legs;
        idx_ESTIMATED_Hard_Contact = idx_ESTIMATED_Slip + robot.model.num_legs;
        idx_ESTIMATED_rpy = idx_ESTIMATED_Hard_Contact + robot.model.num_legs;
        idx_end = idx_ESTIMATED_rpy + 3;
    }

};
#endif //SRC_InEKF_manager_HPP
