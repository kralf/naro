cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
project(naro_sensor_srvs)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE RelWithDebInfo)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy
  nodelet
  std_msgs
  std_srvs
  roscpp_nodewrap
  diagnostic_updater
  message_generation
  naro_usc_srvs
)

################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
# add_message_files(
#    FILES
#    Message1.msg
#    Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
    Calibrate.srv
    GetDepth.srv
    GetElevation.srv
    GetPressure.srv
    GetPosition.srv
    SetFrequency.srv
    GetFrequency.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 #   INCLUDE_DIRS include
 #   LIBRARIES naro_sensor_srvs
    CATKIN_DEPENDS std_msgs std_srvs roscpp_nodewrap diagnostic_updater message_runtime naro_usc_srvs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(
	hall_sensor_node src/HallSensorNode.cpp
)

add_library(
	hall_sensor_class src/HallSensor.cpp
)

add_library(
	gpio_class src/GPIOClass.cpp
)

## Declare a cpp executable
add_executable(
  hall_sensor src/hall_sensor.cpp
)
add_executable(
	depth_sensor src/depth_sensor.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(depth_sensor naro_sensor_srvs_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(
  depth_sensor
  ${catkin_LIBRARIES}
)

target_link_libraries(
  hall_sensor_node
  ${catkin_LIBRARIES}
)
target_link_libraries(
  hall_sensor
  hall_sensor_node
  ${catkin_LIBRARIES}
)
target_link_libraries(
  hall_sensor
  hall_sensor_node
  hall_sensor_class
  gpio_class
  ${catkin_LIBRARIES}
)
