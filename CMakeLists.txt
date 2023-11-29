cmake_minimum_required(VERSION 3.0.2)
project(adi_tmcl)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  message_generation
  std_msgs
)

#################################
## Getting User Configurations ##
#################################

option(TMCL_ROS_DEBUG "Show ROS Debug Stream of tmcl_ros" OFF)
IF(TMCL_ROS_DEBUG)
add_definitions(-DDEBUG)
MESSAGE(STATUS "TMCL_ROS_DEBUG is enabled")
ELSE()
MESSAGE(STATUS "TMCL_ROS_DEBUG is disabled")
ENDIF(TMCL_ROS_DEBUG)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   TmcInfo.msg
   TmcParam.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   TmcCustomCmd.srv
   TmcGapGgpAll.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs 
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    tmcl_ros
    tmcl_interpreter
    ${PROJECT_NAME}_socket_can_wrapper
    tmcl_motor
    tmcl_bldc_motor
    tmcl_stepper_motor
  CATKIN_DEPENDS roscpp message_runtime std_msgs

)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include/tmcl_ros
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_socket_can_wrapper src/socket_can_wrapper.cpp)
add_library(tmcl_interpreter src/tmcl_interpreter.cpp)
add_library(tmcl_ros src/tmcl_ros.cpp)
add_library(tmcl_motor src/tmcl_motor.cpp)
add_library(tmcl_bldc_motor src/tmcl_bldc_motor.cpp)
add_library(tmcl_stepper_motor src/tmcl_stepper_motor.cpp)

add_dependencies(tmcl_ros ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(tmcl_motor ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(tmcl_bldc_motor ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(tmcl_stepper_motor ${PROJECT_NAME}_generate_messages_cpp)

## Add cmake target dependencies of the library
add_dependencies(tmcl_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(tmcl_ros_node src/tmcl_ros_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(tmcl_interpreter ${PROJECT_NAME}_socket_can_wrapper)
target_link_libraries(tmcl_ros tmcl_interpreter)
target_link_libraries(tmcl_ros tmcl_motor)
target_link_libraries(tmcl_interpreter tmcl_motor)
target_link_libraries(tmcl_motor tmcl_bldc_motor)
target_link_libraries(tmcl_motor tmcl_stepper_motor)
target_link_libraries(tmcl_ros_node tmcl_ros ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable
add_dependencies(tmcl_ros_node ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS tmcl_ros_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark libaries for installation
install(TARGETS
  ${PROJECT_NAME}_socket_can_wrapper
  tmcl_interpreter
  tmcl_ros
  tmcl_motor
  tmcl_bldc_motor
  tmcl_stepper_motor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY
  config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY
  scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  FILES_MATCHING PATTERN "*.sh"
)

install(FILES
  LICENSE
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## None