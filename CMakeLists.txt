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


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_COMPILER_LAUNCHER ccache)
set(CMAKE_UNITY_BUILD ON)  # Disabled to avoid symbol conflicts between robot implementations

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

#set(CMAKE_BUILD_TYPE "Debug")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#-DEIGEN_USE_MKL_ALL")
#set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall -g")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -flto -march=native")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -march=native")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -no-pie -march=native -ggdb -Wall \
#    -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
#    -Winit-self -Wmissing-include-dirs -Woverloaded-virtual \
#    -Wshadow -Wsign-promo")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3")


# qmake for PODO
set(TARGET INEKF_Estimator_Test)



find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})  # Add this for compatibility

# Find Pinocchio from robotpkg installation
find_package(PkgConfig REQUIRED)
pkg_check_modules(PINOCCHIO REQUIRED pinocchio)

# message(STATUS "Found Pinocchio version ${PINOCCHIO_VERSION}")
# message(STATUS "Pinocchio include dirs: ${PINOCCHIO_INCLUDE_DIRS}")
# message(STATUS "Pinocchio library dirs: ${PINOCCHIO_LIBRARY_DIRS}")

include_directories(${PINOCCHIO_INCLUDE_DIRS})
link_directories(${PINOCCHIO_LIBRARY_DIRS})

# Find yaml-cpp
find_package(yaml-cpp REQUIRED)
# message(STATUS "Found yaml-cpp")



link_directories(
        /usr/lib
        /usr/local/lib
)


include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${EIGEN3_DIR}
        src
        src/Z1_estimation
        src/Z1_estimation/estimator
        src/Z1_estimation/utility
        src/Z1_estimation/InEKF
)


# NOTE: Building InEKF estimator with robot-specific models
# Robot selection is done via RobotConfig.hpp

file(GLOB_RECURSE INEKF_Estimator_src
    src/Z1_estimation/estimator/InEKF_estimator.cpp
    src/Z1_estimation/estimator/*.hpp
    src/Z1_estimation/csvParser*.cpp
    src/Z1_estimation/*.hpp
    src/Z1_estimation/InEKF/*.cpp
    src/Z1_estimation/utility/*.cpp) # maybe streamline this later


add_library(lib_INEKF_Estimator SHARED ${INEKF_Estimator_src})
# Always compile with -O3 optimization
target_compile_options(lib_INEKF_Estimator PRIVATE -O3)

# Set RPATH to find libraries at runtime
set_target_properties(lib_INEKF_Estimator PROPERTIES
    BUILD_RPATH "${PINOCCHIO_LIBRARY_DIRS}"
    INSTALL_RPATH "${PINOCCHIO_LIBRARY_DIRS}"
)

# Force linkage of pinocchio parsers with --no-as-needed
target_link_options(lib_INEKF_Estimator PRIVATE "LINKER:--no-as-needed")

# Link pinocchio libraries using pkg-config info
target_link_libraries(lib_INEKF_Estimator PUBLIC 
    ${PINOCCHIO_LIBRARIES}
    yaml-cpp
)


file(GLOB INEKF_Estimator_test src/main.cpp)

add_executable(${TARGET} ${INEKF_Estimator_test})

# Set RPATH for executable as well
set_target_properties(${TARGET} PROPERTIES
    BUILD_RPATH "${PINOCCHIO_LIBRARY_DIRS}"
    INSTALL_RPATH "${PINOCCHIO_LIBRARY_DIRS}"
)

target_link_libraries(${TARGET} lib_INEKF_Estimator)



