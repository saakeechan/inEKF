// Run from build/ after linking against INEKF_Estimator_x64.
#include "src/estimatorManager/InEKF_manager.hpp"
#include "src/utility/inekf_save_utils.hpp"
#include <cassert>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <yaml-cpp/yaml.h>

std::vector<std::string> columns(const std::string& line) {
    std::vector<std::string> result;
    std::istringstream stream(line);
    std::string value;
    while (std::getline(stream, value, ',')) result.push_back(value);
    return result;
}

int main(int argc, char** argv) {
    assert(argc == 2); // Temporary output directory supplied by test runner.
    INEKFilter_manager f("../config/initialization_config.yaml", "../config/G1_config.yaml");
    Eigen::Vector<double, 12> covariance = Eigen::Vector<double, 12>::Constant(1e-6);
    Eigen::Vector<double, 16> initial = Eigen::Vector<double, 16>::Zero();
    initial(3) = 1.0;
    f.saved_results_folder = argv[1];

    // Initialization is exported at time zero; the first update follows at dt.
    f.RunCSVStep();
    assert(f.saved_count == 2 && f.time_count == 1);
    InEKF_SaveUtils::write_saved_data_impl(f);
    for (const auto& entry : std::filesystem::directory_iterator(argv[1])) {
        if (entry.path().extension() != ".csv") continue;
        std::ifstream file(entry.path());
        std::string line;
        std::getline(file, line);
        const auto headers = columns(line);
        auto column = [&](const std::string& name) {
            const auto it = std::find(headers.begin(), headers.end(), name);
            assert(it != headers.end());
            return std::distance(headers.begin(), it);
        };
        assert(std::find(headers.begin(), headers.end(), "GT_Slip_leg1") == headers.end());
        std::getline(file, line);
        const auto init = columns(line);
        assert(init.size() == headers.size() && std::stod(init[0]) == 0.0);
        assert(std::isinf(std::stod(init[column("EST_dv_norm_leg1")])));
        std::getline(file, line);
        const auto step = columns(line);
        assert(step.size() == headers.size() && std::stod(step[0]) == f.dt);
        assert(std::stod(step[column("EST_Slip_leg1")]) == f.robot.state.Slip[0]);
        assert(std::stod(step[column("EST_Hard_Contact_leg1")]) == f.robot.state.Hard_Contact[0]);
        assert(!std::getline(file, line));
    }

    // YAML end_count includes that row, including when it equals start_count.
    auto config = YAML::LoadFile("../config/initialization_config.yaml");
    const std::string config_path = std::string(argv[1]) + "/initialization.yaml";
    config["textfile"]["start_count"] = 0;
    config["textfile"]["end_count"] = 1;
    { std::ofstream file(config_path); file << config; }
    f.InitializeFromYAML(config_path);
    f.RunCSVStep();
    assert(f.saved_count == 2 && f.time_count == 1);
    bool end_reached = false;
    try { f.RunCSVStep(); } catch (const std::out_of_range&) { end_reached = true; }
    assert(end_reached);
    config["textfile"]["end_count"] = 0;
    { std::ofstream file(config_path); file << config; }
    f.InitializeFromYAML(config_path);
    assert(f.time_length == 0 && f.saved_count == 1 && f.time_count == 0);

    // A final-row initialization has a valid backward GT difference and no update.
    f.start_index = static_cast<int>(f.CSVData.timestamp.size()) - 1;
    f.time_length = f.start_index;
    f.Initialize(0.02, covariance, initial, 2);
    assert(f.saved_count == 1 && f.time_count == 0);
    assert(std::isfinite(f.SAVE_MATRIX[f.idx_TRUE_dv][0]));
    bool exhausted = false;
    try { f.RunCSVStep(); } catch (const std::out_of_range&) { exhausted = true; }
    assert(exhausted);

    // No previous GT row exists in a single-row input: log unknown DV safely.
    f.CSVData.lfoot_pos_x.resize(1); f.CSVData.lfoot_pos_y.resize(1); f.CSVData.lfoot_pos_z.resize(1);
    f.CSVData.rfoot_pos_x.resize(1); f.CSVData.rfoot_pos_y.resize(1); f.CSVData.rfoot_pos_z.resize(1);
    f.start_index = 0;
    f.Initialize(0.02, covariance, initial, 2);
    assert(std::isinf(f.SAVE_MATRIX[f.idx_TRUE_dv][0]));
    bool capacity = false;
    try { InEKF_SaveUtils::save_onestep_impl(f, f.SAVEMAXCNT); }
    catch (const std::out_of_range&) { capacity = true; }
    assert(capacity);

    // Pending contact-NN output replaces both the kinematic vector and norm once.
    f.csv_mode = false;
    f.contact_nn_enabled = true;
    f.Initialize(0.02, covariance, initial, 2);
    f.NN_foot_velocity_world = Eigen::VectorXd::Constant(3 * f.robot.model.num_legs, 2.0);
    f.dv_norm_nn = Eigen::VectorXd::Constant(f.robot.model.num_legs, std::sqrt(12.0));
    Eigen::VectorXd imu = Eigen::VectorXd::Zero(6); imu(5) = 9.806659;
    Eigen::MatrixXd motors = Eigen::MatrixXd::Zero(2, 29);
    std::vector<bool> contacts(f.robot.model.num_legs, false);
    f.Onestep(imu, StepMeasurements{KinematicsMeasurement{contacts, motors}});
    assert(f.robot.state.d_v.isApprox(f.NN_foot_velocity_world));
    assert(f.robot.state.dv_norm.isApprox(Eigen::VectorXd::Constant(f.robot.model.num_legs, std::sqrt(12.0))));
    assert(f.dv_norm_nn.size() == 0);
    f.Onestep(imu, StepMeasurements{KinematicsMeasurement{contacts, motors}});
    assert(!f.robot.state.d_v.isApprox(f.NN_foot_velocity_world));

    auto state = f.filter.getState();
    state.setVelocity(Eigen::Vector3d(11, 0, 0));
    f.filter.setState(state);
    std::ostringstream diagnostics;
    auto* previous_stderr = std::cerr.rdbuf(diagnostics.rdbuf());
    f.Onestep(imu, StepMeasurements{KinematicsMeasurement{contacts, motors}});
    std::cerr.rdbuf(previous_stderr);
    assert(diagnostics.str().find("Estimator diverged") != std::string::npos);
    assert(f.dt == 0.02 && f.frame_count == f.sliding_window_size - 1);
    assert(f.IMU_Gyro.back().isZero());
    f.Onestep(imu, StepMeasurements{KinematicsMeasurement{contacts, motors}}); // Processing continues after reporting divergence.
}
