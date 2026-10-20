cmake_minimum_required(VERSION 3.16)
project(seb_online LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  # Runtime results are only meaningful for optimised builds.
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# The estimators are not copied: they are compiled from the offline benchmark
# sources of this repository (ros2_ws/src/seb_online -> ../../..).
# REALPATH: the package may be reached through a symlink in a colcon workspace.
get_filename_component(_seb_pkg_dir "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)
set(SEB_REPO_ROOT "${_seb_pkg_dir}/../../.." CACHE PATH "Root of the state_estimation_benchmark repository")
get_filename_component(SEB_REPO_ROOT "${SEB_REPO_ROOT}" ABSOLUTE)

# Pinocchio's exported target enables OpenMP. Keep Eigen single-threaded in every
# target of this package (estimator libraries included) so that an estimator
# update runs on the estimator thread only, as in the offline benchmark build;
# otherwise runtime measurements would include hidden OpenMP parallelism.
add_compile_definitions(EIGEN_DONT_PARALLELIZE)

find_package(ament_cmake REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(seb_interfaces REQUIRED)
find_package(unitree_go REQUIRED)
find_package(dls2_interface REQUIRED)

# ---------------------------------------------------------------------------
# Estimator libraries of the offline benchmark (offline executables not built)
# ---------------------------------------------------------------------------
set(SEB_BUILD_OFFLINE_APPS OFF CACHE BOOL "" FORCE)
add_subdirectory(${SEB_REPO_ROOT}/common ${CMAKE_CURRENT_BINARY_DIR}/offline/common)
add_subdirectory(${SEB_REPO_ROOT}/muse ${CMAKE_CURRENT_BINARY_DIR}/offline/muse)
add_subdirectory(${SEB_REPO_ROOT}/iekf ${CMAKE_CURRENT_BINARY_DIR}/offline/iekf)
add_subdirectory(${SEB_REPO_ROOT}/invariant_smoother ${CMAKE_CURRENT_BINARY_DIR}/offline/invariant_smoother)
if (NOT TARGET seb_kinematics)
  message(FATAL_ERROR "seb_kinematics was not built (Pinocchio not found by common/CMakeLists.txt)")
endif()

# ---------------------------------------------------------------------------
# Core pipeline (no ROS dependency)
# ---------------------------------------------------------------------------
add_library(seb_online_core STATIC
  src/types.cpp
  src/robot_mapping.cpp
  src/acquisition.cpp
  src/foot_kinematics.cpp
  src/synchronizer.cpp
  src/thread_utils.cpp
  src/estimator_worker.cpp
  src/pipeline.cpp
  src/run_logger.cpp
  src/adapters/estimator_factory.cpp
  src/adapters/muse_adapter.cpp
  src/adapters/iekf_adapter.cpp
  src/adapters/smoother_adapter.cpp
)
set_target_properties(seb_online_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(seb_online_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(seb_online_core PUBLIC
  seb_kinematics
  muse_lib
  iekf_lib
  invariant_smoother_lib
  Eigen3::Eigen
  pinocchio::pinocchio
  Threads::Threads
)
target_compile_options(seb_online_core PRIVATE -Wall -Wextra)

# ---------------------------------------------------------------------------
# ROS 2 layer
# ---------------------------------------------------------------------------
set(SEB_ROS_DEPS
  rclcpp rclcpp_lifecycle lifecycle_msgs std_srvs diagnostic_msgs nav_msgs geometry_msgs
  ament_index_cpp seb_interfaces unitree_go dls2_interface)

add_library(seb_online_ros STATIC
  src/node/ros_parameters.cpp
  src/node/ros_output.cpp
)
set_target_properties(seb_online_ros PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(seb_online_ros PUBLIC seb_online_core)
ament_target_dependencies(seb_online_ros PUBLIC ${SEB_ROS_DEPS})

add_executable(online_estimator_node src/node/online_estimator_node.cpp)
target_link_libraries(online_estimator_node seb_online_ros)
ament_target_dependencies(online_estimator_node ${SEB_ROS_DEPS})

add_executable(seb_replay src/tools/bag_replay.cpp)
target_link_libraries(seb_replay seb_online_ros)
ament_target_dependencies(seb_replay ${SEB_ROS_DEPS} rosbag2_cpp)

install(TARGETS online_estimator_node seb_replay
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES ${SEB_REPO_ROOT}/models/go2/go2.urdf DESTINATION share/${PROJECT_NAME}/models)
install(PROGRAMS
  scripts/analyze_run.py
  scripts/inspect_lowstate_bag.py
  scripts/profile_window_sizes.sh
  scripts/compare_offline_regression.py
  scripts/resource_monitor.py
  scripts/benchmark_summary.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  set(SEB_TEST_DEFINITIONS
    SEB_GO2_URDF="${SEB_REPO_ROOT}/models/go2/go2.urdf"
    SEB_ANYMAL_URDF="${SEB_REPO_ROOT}/models/anymalD/anymal.urdf")

  set(SEB_CORE_TESTS
    test_bounded_queue
    test_synchronizer
    test_acquisition
    test_contact_detector
    test_go2_model
    test_pipeline
    test_estimators
  )
  foreach(t ${SEB_CORE_TESTS})
    ament_add_gtest(${t} test/${t}.cpp TIMEOUT 600)
    target_link_libraries(${t} seb_online_core)
    target_include_directories(${t} PRIVATE test)
    target_compile_definitions(${t} PRIVATE ${SEB_TEST_DEFINITIONS})
  endforeach()

  # Conversion from the real unitree_go/LowState message type.
  ament_add_gtest(test_lowstate_message test/test_lowstate_message.cpp)
  target_link_libraries(test_lowstate_message seb_online_core)
  ament_target_dependencies(test_lowstate_message unitree_go)
  target_compile_definitions(test_lowstate_message PRIVATE ${SEB_TEST_DEFINITIONS})

  # Monitoring-only guarantee: no actuation interface in the package sources.
  add_test(NAME test_no_actuation_interface
    COMMAND ${CMAKE_COMMAND} -DSRC_DIR=${CMAKE_CURRENT_SOURCE_DIR} -P ${CMAKE_CURRENT_SOURCE_DIR}/test/check_no_actuation.cmake)
endif()

ament_package()
