cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  naro-ros
  VERSION 0.1
  RELEASE alpha
  SUMMARY "Naro Nanins ROS bindings."
  AUTHOR "Ralf Kaestner"
  CONTACT "ralf.kaestner@gmail.com"
  HOME "http://github.com/kralf/naro"
  LICENSE "GNU Lesser General Public License (LGPL)"
  PREFIX naro-
  INSTALL /usr
)

remake_ros_pack_deb()
