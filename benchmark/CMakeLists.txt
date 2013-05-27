cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  naro-benchmark
  VERSION 0.1
  RELEASE alpha
  SUMMARY "Naro Nanins benchmarking tools."
  AUTHOR "Ralf Kaestner"
  CONTACT "ralf.kaestner@gmail.com"
  HOME "http://github.com/kralf/naro"
  LICENSE "GNU Lesser General Public License (LGPL)"
  PREFIX naro-
  INSTALL /usr
  SCRIPT_DESTINATION /etc/init.d
)

remake_pack_deb()
