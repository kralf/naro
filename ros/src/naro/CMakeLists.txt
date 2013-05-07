remake_ros_package(
  naro_smc_srvs
  DEPENDS roscpp rospy diagnostic_updater
  EXTRA_BUILD_DEPENDS libpololu-dev
  EXTRA_RUN_DEPENDS libpololu
  DESCRIPTION "simple motor controller services"
)

remake_ros_package(
  naro_diagnostics
  RUN_DEPENDS roscpp rospy diagnostic_aggregator
  DESCRIPTION "diagnostics"
)

remake_ros_package(
  naro_test
  DEPENDS roscpp rospy naro_smc_srvs
  DESCRIPTION "testing tools"
)
