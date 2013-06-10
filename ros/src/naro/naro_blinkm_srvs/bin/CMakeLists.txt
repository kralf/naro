remake_find_package(libblinkm CONFIG)
remake_include(${LIBBLINKM_INCLUDE_DIRS})

remake_ros_package_add_executable(blinkm_server LINK ${LIBBLINKM_LIBRARIES})
