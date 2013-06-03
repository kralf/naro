remake_find_package(eigen3 CONFIG)
remake_include(${EIGEN3_INCLUDE_DIRS})

remake_ros_package_add_executable(dive_controller)
