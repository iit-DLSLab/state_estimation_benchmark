cmake_minimum_required(VERSION 3.18)
project(iekf LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

# ---- Sources (include .cpp, .cc, .cxx)
file(GLOB_RECURSE IEKF_SOURCES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/estimator/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/estimator/*.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/estimator/*.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/src/utility/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/utility/*.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/utility/*.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/src/InEKF/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/InEKF/*.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/InEKF/*.cxx
)

add_library(iekf_lib STATIC ${IEKF_SOURCES})

target_include_directories(iekf_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(iekf_lib PUBLIC Eigen3::Eigen)

# (opzionale ma utile)
target_compile_options(iekf_lib PRIVATE -Wall -Wextra -Wpedantic)

# ---- Executable
add_executable(main_iekf ${CMAKE_CURRENT_SOURCE_DIR}/apps/main_iekf.cpp)
target_link_libraries(main_iekf PRIVATE iekf_lib)