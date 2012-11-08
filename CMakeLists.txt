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

rosbuild_add_boost_directories()

#############
# Libraries #
#############

include_directories("${CMAKE_SOURCE_DIR}")
include_directories("${CMAKE_SOURCE_DIR}/src")

include_directories("${CMAKE_SOURCE_DIR}/flashcode_mbt")

# Library gathering libvisp_tracker used by all nodes.
rosbuild_add_library(auto_tracker
  flashcode_mbt/libauto_tracker/states.hpp 
  flashcode_mbt/libauto_tracker/events.h 
  flashcode_mbt/libauto_tracker/tracking.h 
  flashcode_mbt/libauto_tracker/tracking.cpp 
  flashcode_mbt/libauto_tracker/logfilewriter.hpp 
  flashcode_mbt/libauto_tracker/threading.h 
  flashcode_mbt/libauto_tracker/threading.cpp)

add_subdirectory(flashcode_mbt/cmd_line)
add_subdirectory(flashcode_mbt/detectors)


rosbuild_add_library(visp_auto_tracker_nodelet
  src/nodelets/controller.cpp
  )

rosbuild_link_boost(visp_auto_tracker_nodelet
  filesystem system signals regex date_time program_options thread)

target_link_libraries(visp_auto_tracker_nodelet auto_tracker visp-2 auto_tracker qrcode_detector datamatrix_detector cmd_line dmtx zbar )

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
