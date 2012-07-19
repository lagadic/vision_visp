cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)

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

rosbuild_add_boost_directories()

# Make sure Boost.Filesystem v2 is used.
add_definitions(-DBOOST_FILESYSTEM_VERSION=2)

#############
# Libraries #
#############

include_directories("${CMAKE_SOURCE_DIR}/src")
include_directories("${CMAKE_SOURCE_DIR}/src/libvisp_tracker")

# Library gathering libvisp_tracker used by all nodes.
rosbuild_add_library(visp_tracker
  src/libvisp_tracker/conversion.cpp
  src/libvisp_tracker/callbacks.cpp
  src/libvisp_tracker/file.cpp
  src/libvisp_tracker/tracker.cpp
  src/libvisp_tracker/tracker-client.cpp
  src/libvisp_tracker/tracker-viewer.cpp
  src/libvisp_tracker/names.cpp)
target_link_libraries(visp_tracker visp-2)
rosbuild_link_boost(visp_tracker
  filesystem system signals regex date_time program_options thread)

############
# Nodelets #
############

# Tracker, client, viewer nodelets.
rosbuild_add_library(trackerNodelet
  src/nodelets/tracker.cpp
  src/nodelets/client.cpp
  src/nodelets/viewer.cpp)
target_link_libraries(trackerNodelet visp_tracker visp-2)

########
# Node #
########

rosbuild_add_executable(tracker src/nodes/tracker.cpp)
rosbuild_add_executable(client src/nodes/client.cpp)
rosbuild_add_executable(viewer src/nodes/viewer.cpp)

#############
# Bag files #
#############

# Tutorial
rosbuild_download_data(
  https://github.com/downloads/laas/visp_tracker/tutorial-static-box.bag
  bag/tutorial-static-box.bag
  1578dedd48d3f9f5515a8737845ae882)
