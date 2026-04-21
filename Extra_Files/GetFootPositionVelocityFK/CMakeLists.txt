cmake_minimum_required(VERSION 3.10)
project(FootPositionVelocityFK LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO REQUIRED pinocchio)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
    ${EIGEN3_INCLUDE_DIR}
    ${PINOCCHIO_INCLUDE_DIRS}
)

link_directories(${PINOCCHIO_LIBRARY_DIRS})

# Create executable
add_executable(getFootPositionVelocityFK 
    getFootPositionVelocityFK.cpp
    ../csvParserG1_FootPosVel.cpp
    ../../src/utility/RobotModel.cpp
)

# Set RPATH
set_target_properties(getFootPositionVelocityFK PROPERTIES
    BUILD_RPATH "${PINOCCHIO_LIBRARY_DIRS}"
    INSTALL_RPATH "${PINOCCHIO_LIBRARY_DIRS}"
)

# Link libraries
target_link_options(getFootPositionVelocityFK PRIVATE "LINKER:--no-as-needed,-rpath,/opt/openrobots/lib")

target_link_libraries(getFootPositionVelocityFK 
    pinocchio_default
    pinocchio_parsers  
    coal
    boost_filesystem
    boost_serialization
    boost_system
    urdfdom_sensor
    urdfdom_model_state
    urdfdom_model
    urdfdom_world
    console_bridge
    yaml-cpp
)
