#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "estimator/InEKF_estimator.hpp"
#include "utility/RobotModel.hpp"

namespace py = pybind11;

PYBIND11_MODULE(inekf_estimator, m) {
    m.doc() = "Python bindings for InEKF Estimator";

    // Bind ROBOT_STATES struct
    py::class_<ROBOT_STATES>(m, "RobotStates")
        .def(py::init<>())
        .def_readwrite("Rotation", &ROBOT_STATES::Rotation)
        .def_readwrite("Velocity", &ROBOT_STATES::Velocity)
        .def_readwrite("Position", &ROBOT_STATES::Position)
        .def_readwrite("d", &ROBOT_STATES::d)
        .def_readwrite("d_v", &ROBOT_STATES::d_v)
        .def_readwrite("Bias_Gyro", &ROBOT_STATES::Bias_Gyro)
        .def_readwrite("Bias_Acc", &ROBOT_STATES::Bias_Acc)
        .def_readwrite("Contact", &ROBOT_STATES::Contact)
        .def_readwrite("Hard_Contact", &ROBOT_STATES::Hard_Contact)
        .def_readwrite("Slip", &ROBOT_STATES::Slip)
        .def("__repr__", [](const ROBOT_STATES &s) {
            return "<RobotStates: Position=[" + 
                   std::to_string(s.Position[0]) + ", " + 
                   std::to_string(s.Position[1]) + ", " + 
                   std::to_string(s.Position[2]) + "]>";
        });

    // Bind InEKFilter class
    py::class_<InEKFilter>(m, "InEKFilter")
        .def(py::init<>())
        
        // Configuration methods
        .def("load_robot_config", &InEKFilter::load_robot_config,
             py::arg("config_path"),
             "Load robot configuration from YAML file")
        
        .def("Initialize", &InEKFilter::Initialize,
             py::arg("dt"),
             py::arg("covariances"),
             py::arg("initial_condition"),
             py::arg("sliding_window_size"),
             "Initialize the estimator with parameters")
        
        // Main estimation step
        .def("Onestep", &InEKFilter::Onestep,
             py::arg("sensor"),
             py::arg("contact"),
             py::arg("state"),
             "Run one estimation step")
        
        // File I/O methods
        .def("LoadTextFileData", &InEKFilter::LoadTextFileData,
             py::arg("filename"),
             "Load CSV data file for offline processing")
        
        .def("writeSavedData", &InEKFilter::writeSavedData,
             py::arg("info"),
             "Write saved estimation data to file")
        
        // Properties
        .def_readwrite("slip_rejection_mode", &InEKFilter::slip_rejection_mode,
                      "Enable/disable slip rejection mode")
        .def_readwrite("slip_threshold", &InEKFilter::slip_threshold,
                      "Slip threshold in m/s")
        .def_readwrite("variable_contact_cov_mode", &InEKFilter::variable_contact_cov_mode,
                      "Enable/disable variable contact covariance mode")
        .def_readwrite("cov_amplifier", &InEKFilter::cov_amplifier,
                      "Covariance amplifier for contact uncertainty")
        .def_readwrite("TextFile_Skip_Data", &InEKFilter::TextFile_Skip_Data,
                      "Number of initial data points to skip in text file")
        
        // Access to robot model
        .def_readonly("robot", &InEKFilter::robot,
                     "Robot model configuration")
        
        .def("__repr__", [](const InEKFilter &e) {
            return "<InEKFilter: slip_rejection=" + 
                   std::to_string(e.slip_rejection_mode) + ">";
        });

    // Bind RobotModel class
    py::class_<RobotModel>(m, "RobotModel")
        .def(py::init<>())
        .def("load_config", &RobotModel::load_config,
             py::arg("config_path"),
             "Load robot configuration from YAML file")
        .def_readonly("num_legs", &RobotModel::num_legs,
                     "Number of legs")
        .def_readonly("encoder_size", &RobotModel::encoder_size,
                     "Size of encoder vector")
        .def_readonly("encoderdot_size", &RobotModel::encoderdot_size,
                     "Size of encoder velocity vector")
        .def_readonly("textfile_encoder_pos_mapping", 
                     &RobotModel::textfile_encoder_pos_mapping,
                     "Mapping indices for encoder positions from CSV")
        .def_readonly("textfile_encoder_vel_mapping",
                     &RobotModel::textfile_encoder_vel_mapping,
                     "Mapping indices for encoder velocities from CSV")
        .def("__repr__", [](const RobotModel &r) {
            return "<RobotModel: legs=" + std::to_string(r.num_legs) + 
                   " encoder_size=" + std::to_string(r.encoder_size) + ">";
        });

    // Add version info
    m.attr("__version__") = "1.0.0";
}
