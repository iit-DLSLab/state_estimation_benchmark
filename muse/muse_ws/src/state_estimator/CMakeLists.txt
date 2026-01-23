cmake_minimum_required(VERSION 2.8.3)
project(state_estimator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  dynamic_reconfigure
  pluginlib
  message_filters
  message_generation
  state_estimator_msgs
  std_msgs
  rosconsole
  iit_commons
  )

find_package(Boost REQUIRED COMPONENTS system program_options)

find_package(Eigen3 REQUIRED NO_MODULE)

find_package(PkgConfig REQUIRED)

find_package(pinocchio REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
    # RollPitch.msg
    # RollPitchStamped.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   nav_msgs
   state_estimator
   state_estimator_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
#  generate_dynamic_reconfigure_options(
#     #  cfg/HeadBaseTransform.cfg
# #   cfg/DynReconf1.cfg
# #   cfg/DynReconf2.cfg
#  )


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
    ${BOOST_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}_commons ${PROJECT_NAME}_plugins ${PROJECT_NAME}_models 
  CATKIN_DEPENDS pluginlib state_estimator_msgs roscpp iit_commons
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  /usr/include/eigen3  
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

include_directories(${PINOCCHIO_INCLUDE_DIRS})


## Declare a C++ library
add_library(${PROJECT_NAME}_plugins
    src/plugins/attitude_estimation_plugin.cpp
    src/plugins/contact_detection_plugin.cpp
    src/plugins/leg_odometry_plugin.cpp
    src/plugins/sensor_fusion_plugin.cpp
    # src/Models/sensor_fusion.cpp 
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

add_executable(${PROJECT_NAME}_node src/state_estimator.cpp src/state_estimator_node.cpp)

## Rename C++ executable without prefix
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
message("${${PROJECT_NAME}_EXPORTED_TARGETS}")
  add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

target_link_libraries(state_estimator_node
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_plugins
  ${catkin_LIBRARIES}  # <-- this includes message_filters now
  ${PINOCCHIO_LIBRARIES}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  ${PROJECT_NAME}_plugins.xml #${PROJECT_NAME}_tools.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
 
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)


if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    include_directories( include
      ${catkin_INCLUDE_DIRS}
      ${Eigen_INCLUDE_DIRS}
    )

endif()
