cmake_minimum_required(VERSION 3.12)    
project(state_estimator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # throw error if C++17 not supported
set(CMAKE_POSITION_INDEPENDENT_CODE ON) # -fPIC when needed

# Headers for MUSE internals and pinocchio (which is now embedded to simplify stuff)
include_directories(
  "${CMAKE_SOURCE_DIR}/include"
  "${CMAKE_SOURCE_DIR}/muse_ws/src/state_estimator/include"
  "${CMAKE_SOURCE_DIR}/muse_ws/src/iit_commons/include"
  /usr/include/eigen3
)

# Pinocchio from ./lib only (project-local, again to simplify stuff)
find_library(PINOCCHIO_DEFAULT NAMES pinocchio_default
             HINTS "${CMAKE_SOURCE_DIR}/lib" NO_DEFAULT_PATH REQUIRED)
find_library(PINOCCHIO_PARSERS NAMES pinocchio_parsers
             HINTS "${CMAKE_SOURCE_DIR}/lib" NO_DEFAULT_PATH REQUIRED)


# Copy your local Pinocchio libs (from ./lib) next to the built .so after build
file(GLOB PINO_LIBS
  "${CMAKE_SOURCE_DIR}/lib/libpinocchio_default.so.*"
  "${CMAKE_SOURCE_DIR}/lib/libpinocchio_parsers.so.*"
  "${CMAKE_SOURCE_DIR}/muse_conf.txt"
)


set(BIN_DIR "${CMAKE_BINARY_DIR}/bin")
add_executable(Estimator standalone_state_estimator.cpp)
target_compile_options(Estimator PRIVATE -fno-omit-frame-pointer) # Compile: no optimization, full debug info, keep frame pointers for better backtraces/profiling
target_link_libraries(Estimator PRIVATE ${PINOCCHIO_DEFAULT} ${PINOCCHIO_PARSERS})
set_target_properties(Estimator PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${BIN_DIR}"
  BUILD_RPATH "$ORIGIN"
  INSTALL_RPATH "$ORIGIN"
)

add_custom_command(TARGET Estimator POST_BUILD # copy if needed
  COMMAND ${CMAKE_COMMAND} -E make_directory "${BIN_DIR}"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${PINO_LIBS} "${BIN_DIR}/"
  VERBATIM
)
