cmake_minimum_required(VERSION 3.18)
project(muse_offline LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(SEB_BUILD_OFFLINE_APPS "Build the offline CSV benchmark executables" ON)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../common common_build)

find_package(Eigen3 REQUIRED)

# MUSE is header-only: expose it as an interface library so that the offline
# executable and the online ROS 2 estimator use the same sources.
add_library(muse_lib INTERFACE)
target_include_directories(muse_lib SYSTEM INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/include
)
target_link_libraries(muse_lib INTERFACE Eigen3::Eigen)

if (SEB_BUILD_OFFLINE_APPS)
  add_executable(main_muse apps/main_muse.cpp)
  target_link_libraries(main_muse PRIVATE muse_lib)
endif()
