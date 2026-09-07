cmake_minimum_required(VERSION 3.18)
project(data_process LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Import common
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../common common_build)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

add_executable(inspect_dataset
  apps/inspect_dataset.cpp
)
target_link_libraries(inspect_dataset PRIVATE
  seb_common
)

add_executable(precompute_feet_kinematics
  apps/precompute_feet_kinematics.cpp
)
target_link_libraries(precompute_feet_kinematics PRIVATE
  seb_common
  pinocchio::pinocchio
)

# Include paths (if you need more than seb_common)
target_include_directories(inspect_dataset PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/../common/include
)

target_include_directories(precompute_feet_kinematics PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/../common/include
)
