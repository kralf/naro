cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  naro-init
  VERSION 0.1
  RELEASE alpha
  SUMMARY "Naro Nanins system startup scripts."
  AUTHOR "Ralf Kaestner"
  CONTACT "ralf.kaestner@gmail.com"
  HOME "http://github.com/kralf/naro"
  LICENSE "GNU Lesser General Public License (LGPL)"
  PREFIX naro-
  INSTALL /usr
  SCRIPT_DESTINATION /etc
  CONFIGURATION_DESTINATION /etc/naro
)

remake_pack_deb(
  ARCH all
  DEPENDS lsb-base base-files
  EXTRA debian/*
)

remake_distribute_deb(
  DISTRIBUTION lucid
  SECTION admin
  UPLOAD ppa:kralf/naro
  DEPENDS lsb-base base-files remake
  PASS CMAKE_BUILD_TYPE NARO_INIT_GIT_REVISION
)

remake_distribute_deb(
  DISTRIBUTION precise
  SECTION admin
  UPLOAD ppa:kralf/naro
  DEPENDS lsb-base base-files remake
  PASS CMAKE_BUILD_TYPE NARO_INIT_GIT_REVISION
)
