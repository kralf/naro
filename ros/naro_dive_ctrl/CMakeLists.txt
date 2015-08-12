cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
project(naro_dive_ctrl)

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
  naro_sensor_srvs
  naro_tank_ctrl
  naro_imu
  naro_monitoring
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
    Disable.srv
    #Emerge.srv
    EnablePIDCtrl.srv
    EnableLQRCtrl.srv
    #GetActual.srv
    #GetCommand.srv
    #GetEnabled.srv
    #GetError.srv
    #GetGains.srv
    #SetCommand.srv
    SetGains.srv
    SetDepth.srv
    SetPitch.srv
    SetRefState.srv
    GetRefDepth.srv
    GetRefPitch.srv
    SetTankPos.srv
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
 # INCLUDE_DIRS include 
 #   LIBRARIES naro_sensor_srvs
    CATKIN_DEPENDS 
    	std_msgs 
    	std_srvs 
    	roscpp_nodewrap 
    	diagnostic_updater 
    	message_runtime 
    	naro_sensor_srvs
    	naro_tank_ctrl
    	naro_imu
    	naro_monitoring
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
	pidCtrl src/PIDController.cpp
)

add_library(
	diveCtrl src/DiveController.cpp
)

#add_library(
#    logging_class src/LoggingClass.cpp
#)


## Declare a cpp executable
#add_executable(dive_controller_old src/dive_controller.cpp)

add_executable(
	dive_controller src/diveController_node.cpp
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(pidCtrl diveCtrl diveCtrl_node naro_monitoring naro_dive_ctrl_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(
  diveCtrl
  ${catkin_LIBRARIES}
)

target_link_libraries(
  dive_controller
  diveCtrl
  ${catkin_LIBRARIES}
)

target_link_libraries(
  dive_controller
  diveCtrl
  pidCtrl
  ${catkin_LIBRARIES}
)

target_link_libraries(
  dive_controller
  diveCtrl
  pidCtrl
  #logging_class
  ${catkin_LIBRARIES}
)

