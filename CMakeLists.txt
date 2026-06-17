cmake_minimum_required(VERSION 3.10)

project(INEKF_Estimator LANGUAGES CXX)


macro(check_build_type_macro val)
    if(${val} MATCHES Release)
        message("build_type is Release")
    else()
        message("build_type is Debug")
    endif()
endmacro(check_build_type_macro val)

check_build_type_macro(CMAKE_BUILD_TYPE)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER ccache)
endif()
set(CMAKE_UNITY_BUILD OFF)  # Disabled to avoid symbol conflicts between robot implementations

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()


# qmake for PODO
set(TARGET INEKF_Estimator_Test)



find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})  # Add this for compatibility

# Find Pinocchio from robotpkg installation
find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO REQUIRED pinocchio)

include_directories(${PINOCCHIO_INCLUDE_DIRS})
link_directories(${PINOCCHIO_LIBRARY_DIRS})

find_package(yaml-cpp REQUIRED)

# Find ONNX Runtime
find_library(ONNXRUNTIME_LIB onnxruntime
    HINTS /usr/local/lib /usr/lib /opt/onnxruntime/lib
)
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
    HINTS /usr/local/include /usr/include /opt/onnxruntime/include
)

if(NOT ONNXRUNTIME_LIB OR NOT ONNXRUNTIME_INCLUDE_DIR)
    message(FATAL_ERROR "ONNX Runtime not found. Please install ONNX Runtime.")
else()
    message(STATUS "Found ONNX Runtime: ${ONNXRUNTIME_LIB}")
    message(STATUS "ONNX Runtime include: ${ONNXRUNTIME_INCLUDE_DIR}")
endif()

include_directories(${ONNXRUNTIME_INCLUDE_DIR})

link_directories(
        /usr/lib
        /usr/local/lib
)


include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${EIGEN3_DIR}
        src
        src/estimator
        src/utility
        src/InEKF
        src/contactDetection
)


file(GLOB_RECURSE INEKF_Estimator_src
    src/*.cpp
    src/*.hpp)

# Remove files with main() functions from the source list
list (FILTER INEKF_Estimator_src EXCLUDE REGEX ".*/main\\.cpp$")


add_library(INEKF_Estimator SHARED ${INEKF_Estimator_src})
# Always compile with -O3 optimization
target_compile_options(INEKF_Estimator PRIVATE -O3)

# Set RPATH to find libraries at runtime
set_target_properties(INEKF_Estimator PROPERTIES
    BUILD_RPATH "${PINOCCHIO_LIBRARY_DIRS}"
    INSTALL_RPATH "${PINOCCHIO_LIBRARY_DIRS}"
)

# Force linkage of pinocchio parsers with --no-as-needed
target_link_options(INEKF_Estimator PRIVATE "LINKER:--no-as-needed,-rpath,/opt/openrobots/lib")

# Link pinocchio libraries (excluding casadi which has version compatibility issues)
target_link_libraries(INEKF_Estimator PUBLIC 
    pinocchio_default
    pinocchio_parsers
    boost_filesystem
    boost_serialization
    boost_system
    urdfdom_sensor
    urdfdom_model_state
    urdfdom_model
    urdfdom_world
    console_bridge
    yaml-cpp
    ${ONNXRUNTIME_LIB}
)


file(GLOB INEKF_Estimator_test src/main.cpp)

add_executable(${TARGET} ${INEKF_Estimator_test})

# Set RPATH for executable as well
set_target_properties(${TARGET} PROPERTIES
    BUILD_RPATH "${PINOCCHIO_LIBRARY_DIRS}"
    INSTALL_RPATH "${PINOCCHIO_LIBRARY_DIRS}"
)

target_link_libraries(${TARGET} INEKF_Estimator)

# writeFKFootPosVel executable: Compute FK foot positions/velocities and append to CSV
add_executable(writeFKFootPosVel 
    src/utility/writeFKFootPosVel.cpp
    src/utility/csvParser.cpp
)

set_target_properties(writeFKFootPosVel PROPERTIES
    BUILD_RPATH "${PINOCCHIO_LIBRARY_DIRS}"
    INSTALL_RPATH "${PINOCCHIO_LIBRARY_DIRS}"
)

target_link_libraries(writeFKFootPosVel
    INEKF_Estimator
    pinocchio_default
    pinocchio_parsers
    yaml-cpp
)

# DynKinSlip executable: Dynamic Kinematic Slip Analysis for left foot
add_executable(DynKinSlip 
    Extra_Files/DynKinSlipAnalysis/DynKinSlip.cpp
    src/utility/csvParser.cpp
)

set_target_properties(DynKinSlip PROPERTIES
    BUILD_RPATH "${PINOCCHIO_LIBRARY_DIRS}"
    INSTALL_RPATH "${PINOCCHIO_LIBRARY_DIRS}"
)

target_link_libraries(DynKinSlip
    INEKF_Estimator
    pinocchio_default
    pinocchio_parsers
    yaml-cpp
)


# ============================================================
# Installation rules
# ============================================================

# Install header files
install(DIRECTORY src/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/inekf_estimator
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)