cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()
rosbuild_gensrv()

# Dynamic reconfigure.
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

# Generate names.cpp.
configure_file(
  "${CMAKE_SOURCE_DIR}/src/names.cpp.in"
  "${CMAKE_BINARY_DIR}/src/names.cpp"
  @ONLY)
include_directories("${CMAKE_SOURCE_DIR}/src")

# Library gathering tools used by both server and client.
rosbuild_add_library(tools
  src/conversion.cpp
  src/callbacks.cpp
  src/file.cpp
  "${CMAKE_BINARY_DIR}/src/names.cpp")
target_link_libraries(tools visp-2)

# Node
rosbuild_add_executable(tracker_mbt src/tracker_mbt.cpp)
target_link_libraries(tracker_mbt tools visp-2)

# Client
rosbuild_add_executable(tracker_mbt_client src/tracker_mbt_client.cpp)
target_link_libraries(tracker_mbt_client tools visp-2)


# Viewer
rosbuild_add_executable(tracker_mbt_viewer src/tracker_mbt_viewer.cpp)
target_link_libraries(tracker_mbt_viewer tools visp-2)

# Tutorial
rosbuild_download_data(
  https://github.com/downloads/thomas-moulard/ros-visp_tracker/tutorial-static-box.bag
  bag/tutorial-static-box.bag
  1578dedd48d3f9f5515a8737845ae882)
