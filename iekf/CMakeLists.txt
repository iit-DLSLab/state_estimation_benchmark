cmake_minimum_required(VERSION 3.18)
project(iekf LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

add_executable(main_iekf apps/main_iekf.cpp)

target_include_directories(main_iekf PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(main_iekf PRIVATE
  Eigen3::Eigen
)

# (opzionale) install
# install(TARGETS main_iekf RUNTIME DESTINATION bin)
