cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  naro-udev
  VERSION 0.1
  RELEASE alpha
  SUMMARY "Naro Nanins auto-configuration rules for udev."
  AUTHOR "Ralf Kaestner"
  CONTACT "ralf.kaestner@gmail.com"
  HOME "http://github.com/kralf/naro"
  LICENSE "GNU Lesser General Public License (LGPL)"
  PREFIX naro-
  INSTALL /usr
  CONFIGURATION_DESTINATION /etc/udev
)

remake_add_directories(rules)

remake_pack_deb(
  ARCH all
  DEPENDS udev
  EXTRA debian/*
)

remake_distribute_deb(
  DISTRIBUTION lucid
  SECTION admin
  UPLOAD ppa:kralf/naro
  DEPENDS remake
  PASS CMAKE_BUILD_TYPE NARO_UDEV_GIT_REVISION
)

remake_distribute_deb(
  DISTRIBUTION precise
  SECTION admin
  UPLOAD ppa:kralf/naro
  DEPENDS remake
  PASS CMAKE_BUILD_TYPE NARO_UDEV_GIT_REVISION
)
