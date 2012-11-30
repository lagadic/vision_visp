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

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

include_directories("${CMAKE_SOURCE_DIR}/flashcode_mbt")
include_directories("${CMAKE_SOURCE_DIR}/flashcode_mbt/detectors")

# Library gathering libvisp_tracker used by all nodes.
rosbuild_add_library(auto_tracker
  flashcode_mbt/libauto_tracker/states.hpp
  flashcode_mbt/libauto_tracker/events.h
  flashcode_mbt/libauto_tracker/tracking.h
  flashcode_mbt/libauto_tracker/tracking.cpp
  flashcode_mbt/libauto_tracker/logfilewriter.hpp
  flashcode_mbt/libauto_tracker/threading.h
  flashcode_mbt/libauto_tracker/threading.cpp)

rosbuild_add_library(cmd_line
	flashcode_mbt/cmd_line/cmd_line.cpp
)

add_subdirectory(flashcode_mbt/detectors)

rosbuild_add_executable(visp_auto_tracker
  src/main.cpp
  src/node.cpp
  src/names.cpp
  )

#rosbuild_add_library(controller src/controller/controller_client.cpp)

rosbuild_link_boost(visp_auto_tracker
  filesystem system signals regex date_time program_options thread)

target_link_libraries(visp_auto_tracker auto_tracker qrcode_detector datamatrix_detector cmd_line dmtx zbar )

#############
# Bag files #
#############

# Tutorial
rosbuild_download_data(
  https://github.com/downloads/lagadic/visp_auto_tracker/tutorial-qrcode.bag
  bag/tutorial-qrcode.bag
  0f80ceea2610b8400591ca7aff764dfa)
