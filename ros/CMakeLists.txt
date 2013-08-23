cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  naro-ros-${ROS_DISTRIBUTION}
  VERSION 0.1
  RELEASE alpha
  SUMMARY "Naro Nanins ROS bindings."
  AUTHOR "Ralf Kaestner"
  CONTACT "ralf.kaestner@gmail.com"
  HOME "http://github.com/kralf/naro"
  LICENSE "GNU Lesser General Public License (LGPL)"
  PREFIX naro-
  INSTALL /usr
  CONFIGURATION_DESTINATION /etc/naro
)

remake_ros_pack_deb()

# remake_var_name(GIT_REVISION_VAR ${REMAKE_PROJECT_FILENAME} GIT_REVISION)
# 
# if(ROS_DISTRIBUTION STRLESS "groovy")
#   remake_ros_distribute_deb(
#     DISTRIBUTION lucid
#     SECTION misc
#     ARCH "i386 amd64"
#     UPLOAD ppa:kralf/naro
#     DEPENDS remake pkg-config
#     PASS CMAKE_BUILD_TYPE ${GIT_REVISION_VAR}
#     FORCE_CONSISTENCY
#   )
# endif(ROS_DISTRIBUTION STRLESS "groovy")
# 
# remake_ros_distribute_deb(
#   DISTRIBUTION precise
#   SECTION misc
#   ARCH "i386 amd64"
#   UPLOAD ppa:kralf/naro
#   DEPENDS remake pkg-config
#   PASS CMAKE_BUILD_TYPE ${GIT_REVISION_VAR}
#   FORCE_CONSISTENCY
# )
