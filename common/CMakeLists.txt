cmake_minimum_required(VERSION 3.18)

# This directory is pulled in with add_subdirectory() by several projects
# (data_process, muse, the ROS 2 package); define its targets only once.
if (TARGET seb_common)
  return()
endif()

project(seb_common LANGUAGES CXX)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

add_library(seb_common STATIC
  src/benchmark/DatasetIO.cpp
)
set_target_properties(seb_common PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(seb_common PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(seb_common PUBLIC
  Eigen3::Eigen
)

# (opzionale) warning basic
if (MSVC)
  target_compile_options(seb_common PRIVATE /W4)
else()
  target_compile_options(seb_common PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Pinocchio foot kinematics shared by data_process (offline precomputation) and
# the online ROS 2 kinematics worker. Only built when Pinocchio is available.
find_package(pinocchio QUIET)
if (pinocchio_FOUND)
  add_library(seb_kinematics STATIC
    src/benchmark/FootKinematics.cpp
  )
  set_target_properties(seb_kinematics PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_include_directories(seb_kinematics PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(seb_kinematics PUBLIC
    Eigen3::Eigen
    pinocchio::pinocchio
  )
endif()
