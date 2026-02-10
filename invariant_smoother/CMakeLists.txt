cmake_minimum_required(VERSION 3.18)
project(iekf LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

add_executable(main_invariant_smoother apps/main_invariant_smoother.cpp)

target_include_directories(main_invariant_smoother PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(main_invariant_smoother PRIVATE
  Eigen3::Eigen
)

# (opzionale) install
# install(TARGETS main_invariant_smoother RUNTIME DESTINATION bin)