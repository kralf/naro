remake_ros_package(
  naro_smc_srvs
  DEPENDS roscpp rospy diagnostic_updater
  EXTRA_BUILD_DEPENDS libpololu-dev
  EXTRA_RUN_DEPENDS libpololu
  DESCRIPTION "simple motor controller services"
)

remake_ros_package(
  naro_usc_srvs
  DEPENDS roscpp rospy diagnostic_updater
  EXTRA_BUILD_DEPENDS libpololu-dev
  EXTRA_RUN_DEPENDS libpololu
  DESCRIPTION "USB servo controller services"
)

remake_ros_package(
  naro_blinkm_srvs
  DEPENDS roscpp rospy diagnostic_updater
  EXTRA_BUILD_DEPENDS libblinkm-dev
  EXTRA_RUN_DEPENDS libblinkm
  DESCRIPTION "smart LED services"
)

remake_ros_package(
  naro_sensor_srvs
  DEPENDS roscpp rospy diagnostic_updater naro_usc_srvs
  DESCRIPTION "sensor services"
)

remake_ros_package(
  naro_dive_ctrl
  DEPENDS roscpp rospy diagnostic_updater naro_smc_srvs naro_usc_srvs
  EXTRA_BUILD_DEPENDS libeigen3-dev
  DESCRIPTION "dive controller"
)

remake_ros_package(
  naro_fin_ctrl
  DEPENDS roscpp rospy diagnostic_updater naro_usc_srvs
  DESCRIPTION "fin controller"
)

remake_ros_package(
  naro_diagnostics
  DEPENDS roscpp rospy diagnostic_aggregator
  EXTRA_RUN_DEPENDS sysstat
  DESCRIPTION "diagnostics"
)

remake_ros_package(
  naro_test
  DEPENDS roscpp rospy std_msgs naro_smc_srvs naro_usc_srvs
    naro_sensor_srvs naro_fin_ctrl naro_dive_ctrl
  DESCRIPTION "testing tools"
)
