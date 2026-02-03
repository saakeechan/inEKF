#include <iostream>
#include <iomanip>
#include <fstream>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/mjcf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

int main() {
    // Robot model parameters
    std::string urdf_path = "../../../unitree_ros/robots/go2_description/urdf/go2_description.urdf";
    std::string robot_name = "go2";
    
    pinocchio::Model model;
    
    std::cout << "Loading model: " << urdf_path << std::endl;
    
    // Check file extension to determine format
    std::string extension = "";
    size_t dot_pos = urdf_path.find_last_of(".");
    if (dot_pos != std::string::npos) {
        extension = urdf_path.substr(dot_pos);
    }
    
    // Load model based on file format
    if (extension == ".urdf") {
        std::cout << "Detected URDF format" << std::endl;
        pinocchio::urdf::buildModel(urdf_path, model);
    } else if (extension == ".xml" || extension == ".mjcf") {
        std::cout << "Detected MJCF format" << std::endl;
        pinocchio::mjcf::buildModel(urdf_path, model);
    } else {
        std::cerr << "Error: Unknown file format '" << extension << "'. Supported formats: .urdf, .xml, .mjcf" << std::endl;
        return 1;
    }
    
    pinocchio::Data data(model);
    
    // Open output file
    std::string output_filename = robot_name + "Parameters";
    std::ofstream outfile(output_filename);
    if (!outfile.is_open()) {
        std::cerr << "Error: Could not open file " << output_filename << " for writing." << std::endl;
        return 1;
    }
    
    std::cout << "Writing parameters to: " << output_filename << std::endl;
    
    outfile << "============================================================" << std::endl;
    outfile << "Model Information:" << std::endl;
    outfile << "============================================================" << std::endl;
    outfile << "Number of joints (nq): " << model.nq << std::endl;
    outfile << "Number of velocities (nv): " << model.nv << std::endl;
    outfile << "Number of bodies: " << model.nbodies << std::endl;
    outfile << "Number of frames: " << model.nframes << std::endl;
    
    outfile << "\n============================================================" << std::endl;
    outfile << "Joint Information:" << std::endl;
    outfile << "============================================================" << std::endl;
    for (size_t i = 0; i < model.names.size(); ++i) {
        outfile << std::setw(2) << i << ". " << model.names[i] << std::endl;
    }
    
    outfile << "\n============================================================" << std::endl;
    outfile << "Configuration Vector (q) Mapping:" << std::endl;
    outfile << "============================================================" << std::endl;
    outfile << "Total configuration size: " << model.nq << std::endl;
    outfile << "\nq index → Joint mapping:" << std::endl;
    
    for (size_t i = 1; i < model.joints.size(); ++i) {
        int nq_idx = model.joints[i].idx_q();
        int nv_idx = model.joints[i].idx_v();
        int nq_size = model.joints[i].nq();
        int nv_size = model.joints[i].nv();
        
        outfile << "  q[" << nq_idx;
        if (nq_size > 1) {
            outfile << ":" << (nq_idx + nq_size);
        }
        outfile << "]  → " << model.names[i] 
                  << " (" << model.joints[i].shortname() << ")" << std::endl;
    }
    
    outfile << "\n============================================================" << std::endl;
    outfile << "Velocity Vector (v) Mapping:" << std::endl;
    outfile << "============================================================" << std::endl;
    outfile << "Total velocity size: " << model.nv << std::endl;
    outfile << "\nv index → Joint mapping:" << std::endl;
    
    for (size_t i = 1; i < model.joints.size(); ++i) {
        int nv_idx = model.joints[i].idx_v();
        int nv_size = model.joints[i].nv();
        
        outfile << "  v[" << nv_idx;
        if (nv_size > 1) {
            outfile << ":" << (nv_idx + nv_size);
        }
        outfile << "]  → " << model.names[i] 
                  << " (" << model.joints[i].shortname() << ")" << std::endl;
    }
    
    outfile << "\n============================================================" << std::endl;
    outfile << "\n============================================================" << std::endl;
    outfile << "All Frame Indices:" << std::endl;
    outfile << "============================================================" << std::endl;
    for (size_t i = 0; i < model.nframes; ++i) {
        outfile << "  Frame[" << std::setw(2) << i << "]: " 
                  << std::left << std::setw(30) << model.frames[i].name 
                  << "(parent joint: " << model.names[model.frames[i].parentJoint] << ")" << std::endl;
    }
    
    outfile << "\n============================================================" << std::endl;
    outfile << "Joint Name Mapping (nq vs nv indices):" << std::endl;
    outfile << "============================================================" << std::endl;
    outfile << std::left << std::setw(10) << "Joint ID" 
              << std::setw(30) << "Joint Name" 
              << std::setw(16) << "nq_idx" 
              << std::setw(16) << "nv_idx"
              << std::setw(11) << "nq_size"
              << std::setw(11) << "nv_size" << std::endl;
    outfile << std::string(100, '-') << std::endl;
    
    for (size_t i = 0; i < model.joints.size(); ++i) {
        outfile << std::left << std::setw(10) << i
                  << std::setw(30) << model.names[i]
                  << std::setw(16) << model.joints[i].idx_q()
                  << std::setw(16) << model.joints[i].idx_v()
                  << std::setw(11) << model.joints[i].nq()
                  << std::setw(11) << model.joints[i].nv() << std::endl;
    }
    outfile << "============================================================" << std::endl;
    
    outfile.close();
    std::cout << "Parameters written successfully to: " << output_filename << std::endl;
    
    return 0;
}
