cmake_minimum_required(VERSION 3.18)
project(invariant_smoother LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

# ---- Build the library (all the implementation .cpp)
file(GLOB IS_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/estimator/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/utility/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/factor/*.cpp
)

add_library(invariant_smoother_lib STATIC ${IS_SOURCES})

target_include_directories(invariant_smoother_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(invariant_smoother_lib PUBLIC
  Eigen3::Eigen
)

# ---- Build your executable
add_executable(main_invariant_smoother apps/main_invariant_smoother.cpp)

target_include_directories(main_invariant_smoother PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(main_invariant_smoother PRIVATE
  invariant_smoother_lib
  Eigen3::Eigen
)
