remake_find_package(libpololu CONFIG)
remake_include(${LIBPOLOLU_INCLUDE_DIRS})

remake_ros_package_add_executable(usc_server LINK ${LIBPOLOLU_LIBRARIES})
