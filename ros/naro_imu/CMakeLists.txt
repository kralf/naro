cmake_minimum_required(VERSION 2.8.3)
project(naro_imu)

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
  tf
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################


## Generate services in the 'srv' folder
add_service_files(
  FILES
    GetPitch.srv
)

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
    INCLUDE_DIRS include
    LIBRARIES naro_imu
    CATKIN_DEPENDS std_msgs std_srvs roscpp_nodewrap message_runtime 
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
add_library(naro_imu
  src/NaroImu.cpp
)

## Declare a cpp executable
add_executable(
  naro_imu_node
  src/naro_imu_start.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(
  naro_imu
  ${catkin_LIBRARIES}
)

target_link_libraries(
  naro_imu_node
  naro_imu
  ${catkin_LIBRARIES}
)

## Add cmake target dependencies of the executable/library
add_dependencies(naro_imu naro_imu_generate_messages_cpp)
