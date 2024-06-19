cmake_minimum_required(VERSION 3.0.2)
project(wrist_control)

##
# If not, gives a problem with including dynamixel_sdk.h
include_directories(/home/ordavidro/wrist_ws/src/DynamixelSDK/ros/dynamixel_sdk/include/dynamixel_sdk)
include_directories(/home/ordavidro/wrist_ws/src/dynamixel_ros_library/package/dynamixel_ros_library/include)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamixel_ros_library
  dynamixel_sdk
  dynamixel_workbench_controllers
  dynamixel_workbench_msgs
  dynamixel_workbench_operators
  dynamixel_workbench_toolbox
  message_generation
  message_runtime
  roscpp
  std_msgs
)

##
catkin_package(
  INCLUDE_DIRS ${catkin_INCLUDE_DIRS} ${${PROJECT_NAME}_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_DEPENDS}
)


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 )

## Declare a C++ executable
add_executable(wrist_orientation src/wrist_orientation.cpp)

## Add cmake target dependencies of the executable
add_dependencies(wrist_orientation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(wrist_orientation
  ${catkin_LIBRARIES}
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")


#############
## Install ##
#############

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
 install(TARGETS wrist_orientation
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )



############
# Testing ##
############

# Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_wrist_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()