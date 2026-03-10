#ifndef GET_PARAMETERS_HPP
#define GET_PARAMETERS_HPP

#include <string>
    
/**
 * @brief Extract and write robot model parameters to a file
 * @param urdf_path Path to the robot model file (URDF, MJCF, or XML)
 * @param robot_name Name prefix for the output file
 */
void extractParameters(const std::string& urdf_path, const std::string& robot_name);


#endif // GET_PARAMETERS_HPP
