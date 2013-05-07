remake_find_library(pololu pololu.h PACKAGE libpololu)
remake_include(${POLOLU_HEADERS})

remake_ros_package_add_executable(smc_server LINK ${POLOLU_LIBRARY})
