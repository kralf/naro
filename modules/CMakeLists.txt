cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  naro-kernel-source
  VERSION 0.1
  RELEASE alpha
  SUMMARY "Naro Nanins kernel module sources."
  AUTHOR "Ralf Kaestner"
  CONTACT "ralf.kaestner@gmail.com"
  HOME "http://github.com/kralf/naro"
  LICENSE "GNU Lesser General Public License (LGPL)"
  PREFIX naro-
  INSTALL /usr
)

remake_pack_deb(
  ARCH all
  DEPENDS dkms
  EXTRA debian/*
)

remake_distribute_deb(
  DISTRIBUTION precise
  ALIAS raspbian
  ARCH armhf
  SECTION kernel
  UPLOAD ppa:kralf/naro
  DEPENDS dkms remake
  PASS CMAKE_BUILD_TYPE NARO_KERNEL_SOURCE_GIT_REVISION
)
