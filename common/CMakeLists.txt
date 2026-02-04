cmake_minimum_required(VERSION 3.18)
project(seb_common LANGUAGES CXX)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

add_library(seb_common STATIC
  src/benchmark/DatasetIO.cpp
)

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
