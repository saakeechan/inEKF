//
// Robot Model Configuration
// 
// To use this code with your robot:
// 1. Create your Robot_Model_YourRobot.hpp file in this directory
// 2. Uncomment ONLY ONE line below (or add your own)
//

#ifndef ROBOT_CONFIG_HPP
#define ROBOT_CONFIG_HPP

// ============================================
// SELECT YOUR ROBOT MODEL (uncomment ONE):
// ============================================

#define USE_ROBOT_DIGIT
// #define USE_ROBOT_MINI_CHEETAH
// #define USE_ROBOT_G1
// #define USE_ROBOT_GO2

// Add your robot here:
// #define USE_ROBOT_YOURROBOT


// ============================================
// Auto-include based on selection above
// (Do not modify below unless adding new robot)
// ============================================

#ifdef USE_ROBOT_DIGIT
    #include "Digit/Robot_Model_Digit.hpp"
    #include "Digit/DigitCSVData.hpp"
    #include "Digit/csvParserDigit.hpp"
    using Robot_Model = Robot_Model_Digit;
    using RobotCSVData = DigitCSVData;
    inline RobotCSVData ParseRobotCSV(const std::string& filename) { return ParseDigitCSV(filename); }
    #define ROBOT_NAME "Digit"

#elif defined(USE_ROBOT_MINI_CHEETAH)
    #include "Robot_Model_mini_cheetah.hpp"
    using Robot_Model = Robot_Model_mini_cheetah;
    #define ROBOT_NAME "Mini_Cheetah"

#elif defined(USE_ROBOT_G1)
    #include "G1/Robot_Model_G1.hpp"
    #include "G1/G1CSVData.hpp"
    #include "G1/csvParserG1.hpp"
    using Robot_Model = Robot_Model_G1;
    using RobotCSVData = G1CSVData;
    inline RobotCSVData ParseRobotCSV(const std::string& filename) { return ParseG1CSV(filename); }
    #define ROBOT_NAME "G1"

#elif defined(USE_ROBOT_GO2)
    #include "Go2/Robot_Model_Go2.hpp"
    #include "Go2/Go2CSVData.hpp"
    #include "Go2/csvParserGo2.hpp"
    using Robot_Model = Robot_Model_Go2;
    using RobotCSVData = Go2CSVData;
    inline RobotCSVData ParseRobotCSV(const std::string& filename) { return ParseGo2CSV(filename); }
    #define ROBOT_NAME "Go2"

// Add your robot's include here following the same pattern:
// #elif defined(USE_ROBOT_YOURROBOT)
//     #include "Robot_Model_YourRobot.hpp"
//     using Robot_Model = Robot_Model_YourRobot;
//     #define ROBOT_NAME "YourRobot"

#else
    #error "No robot model selected! Please uncomment ONE robot in RobotConfig.hpp"
#endif

#endif // ROBOT_CONFIG_HPP
