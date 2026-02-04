cmake_minimum_required(VERSION 3.18)
project(muse_offline LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../common common_build)

find_package(Eigen3 REQUIRED)

# add_executable(main_muse
#   apps/main_muse.cpp
# )

# target_link_libraries(main_muse PRIVATE
#   seb_common
#   Eigen3::Eigen
# )

add_executable(main_attitude apps/main_attitude.cpp)
target_link_libraries(main_attitude PRIVATE Eigen3::Eigen)


target_include_directories(main_attitude PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/../common/include
  ${CMAKE_CURRENT_LIST_DIR}/include
)
