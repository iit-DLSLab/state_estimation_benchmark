cmake_minimum_required(VERSION 3.0)
project(smoother_offline)

## Find Eigen
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()
find_package(yaml-cpp REQUIRED)
# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

## Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
)

## Set source files
set(SOURCES
    src/main.cpp
    # src/Models/InvariantSmoother.cpp
    src/Models/invariant_factors.cpp
    src/Models/invariant_smoother.cpp
    src/Models/LeggedRobotKinematics.cpp
    src/Models/smoother_struct.cpp
    src/Models/InvariantExtendedKalmanFilter.cpp
    src/InEKF/InEKF.cpp
    src/InEKF/LieGroup.cpp
    src/InEKF/NoiseParams.cpp
    src/InEKF/RobotState.cpp
    # Add additional source files here if needed
)

## Add an executable
add_executable(smoother_offline ${SOURCES})

## Link Eigen library
target_link_libraries(smoother_offline Eigen3::Eigen yaml-cpp)

target_compile_options(smoother_offline PRIVATE -O3 -march=native )
