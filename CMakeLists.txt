cmake_minimum_required(VERSION 3.10)

find_program(CLANGXX_PROGRAM clang++)
if(CLANGXX_PROGRAM AND (NOT DEFINED CMAKE_CXX_COMPILER OR CMAKE_CXX_COMPILER STREQUAL "/usr/bin/c++"))
    set(CMAKE_CXX_COMPILER "${CLANGXX_PROGRAM}" CACHE FILEPATH "C++ compiler" FORCE)
endif()

project(INEKF_Estimator LANGUAGES CXX)

# Select the deployment architecture.  Keep x64 as the default for existing
# desktop builds; use -DARCH=arm64 for the hardware library.
set(ARCH "x64" CACHE STRING "Target architecture (x64 or arm64)")
set_property(CACHE ARCH PROPERTY STRINGS x64 arm64)


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
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}" CACHE FILEPATH "C++ compiler launcher" FORCE)
endif()
message(STATUS "C++ compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "C++ compiler launcher: ${CMAKE_CXX_COMPILER_LAUNCHER}")
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

if(ARCH STREQUAL "x64")
    set(ONNXRUNTIME_DIR "/usr/local")
    set(ONNXRUNTIME_LIB_FILE "/usr/local/lib/libonnxruntime.so.1.25.1")
    set(ONNXRUNTIME_RPATH "/usr/local/lib")
    set(INEKF_LIB_NAME "INEKF_Estimator_x64")
elseif(ARCH STREQUAL "arm64")
    set(ONNXRUNTIME_DIR "${PROJECT_SOURCE_DIR}/../../../thirdparty/onnxruntime-linux-aarch64-1.24.2")
    set(ONNXRUNTIME_LIB_FILE "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so.1.24.2")
    set(ONNXRUNTIME_RPATH "${ONNXRUNTIME_DIR}/lib")
    set(INEKF_LIB_NAME "INEKF_Estimator_arm64")
else()
    message(FATAL_ERROR "Unsupported ARCH='${ARCH}'. Use x64 or arm64.")
endif()

# robotpkg's x64 package exposes separate default and parser libraries.  The
# ARM64 image builds upstream Pinocchio, whose pkg-config metadata supplies
# the appropriate library list.
if(ARCH STREQUAL "x64")
    set(PINOCCHIO_LINK_LIBS pinocchio_default pinocchio_parsers)
else()
    set(PINOCCHIO_LINK_LIBS ${PINOCCHIO_LIBRARIES})
endif()

if(NOT EXISTS "${ONNXRUNTIME_LIB_FILE}" OR
   NOT EXISTS "${ONNXRUNTIME_DIR}/include/onnxruntime_cxx_api.h")
    message(FATAL_ERROR
        "ONNX Runtime for ARCH=${ARCH} was not found at ${ONNXRUNTIME_DIR}.\n"
        "Expected ${ONNXRUNTIME_LIB_FILE}")
endif()

message(STATUS "Building for ARCH=${ARCH}")
message(STATUS "ONNX Runtime: ${ONNXRUNTIME_DIR}")
include_directories("${ONNXRUNTIME_DIR}/include")

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
    BUILD_RPATH "${PINOCCHIO_LIBRARY_DIRS};${ONNXRUNTIME_RPATH}"
    INSTALL_RPATH "${PINOCCHIO_LIBRARY_DIRS};${ONNXRUNTIME_RPATH}"
    OUTPUT_NAME "${INEKF_LIB_NAME}"
)

# Force linkage of pinocchio parsers with --no-as-needed
target_link_options(INEKF_Estimator PRIVATE "LINKER:--no-as-needed,-rpath,/opt/openrobots/lib")

target_link_libraries(INEKF_Estimator PUBLIC 
    ${PINOCCHIO_LINK_LIBS}
    boost_filesystem
    boost_serialization
    boost_system
    urdfdom_sensor
    urdfdom_model_state
    urdfdom_model
    urdfdom_world
    console_bridge
    yaml-cpp
    "${ONNXRUNTIME_LIB_FILE}"
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
    ${PINOCCHIO_LINK_LIBS}
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
    ${PINOCCHIO_LINK_LIBS}
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
