cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

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
include_directories("${CMAKE_SOURCE_DIR}/flashcode_mbt/detectors")

# Library gathering libvisp_tracker used by all nodes.
rosbuild_add_library(auto_tracker
  flashcode_mbt/libauto_tracker/states.hpp 
  flashcode_mbt/libauto_tracker/events.h 
  flashcode_mbt/libauto_tracker/tracking.h 
  flashcode_mbt/libauto_tracker/tracking.cpp
#  flashcode_mbt/libauto_tracker/tracking_events.cpp  
  flashcode_mbt/libauto_tracker/logfilewriter.hpp 
  flashcode_mbt/libauto_tracker/threading.h 
  flashcode_mbt/libauto_tracker/threading.cpp)

#rosbuild_add_library(tracking_events_visp
#			flashcode_mbt/tracking_events/display_visp.cpp
#			flashcode_mbt/tracking_events/display_visp.h 
#			)
			
add_subdirectory(flashcode_mbt/cmd_line)
add_subdirectory(flashcode_mbt/detectors)

rosbuild_add_library(visp_auto_tracker_nodelet
  src/nodelets/controller.cpp
  )

#rosbuild_add_library(controller src/controller/controller_client.cpp)

rosbuild_link_boost(visp_auto_tracker_nodelet
  filesystem system signals regex date_time program_options thread)

target_link_libraries(visp_auto_tracker_nodelet auto_tracker  visp-2 auto_tracker qrcode_detector datamatrix_detector cmd_line dmtx zbar )
#to be removed
target_link_libraries(visp_auto_tracker_nodelet
/usr/lib/liblapack.so 
/usr/lib/libblas.so 
/usr/lib/i386-linux-gnu/libSM.so 
/usr/lib/i386-linux-gnu/libICE.so 
/usr/lib/i386-linux-gnu/libX11.so 
/usr/lib/i386-linux-gnu/libXext.so 
m 
/usr/lib/libboost_serialization-mt.so 
/usr/lib/libboost_thread-mt.so 
pthread /usr/lib/libboost_date_time-mt.so 
/usr/lib/libOgreMain.so 
/usr/lib/libOgreMain.so 
/usr/lib/libOIS.so 
/usr/lib/libOIS.so 
/usr/lib/i386-linux-gnu/libGLU.so 
/usr/lib/i386-linux-gnu/libGL.so 
/usr/lib/i386-linux-gnu/libSM.so 
/usr/lib/i386-linux-gnu/libICE.so 
/usr/lib/i386-linux-gnu/libX11.so 
/usr/lib/i386-linux-gnu/libXext.so 
/usr/lib/libCoin.so 
/usr/lib/libxml2.so 
/usr/lib/libfreenect.so 
/usr/lib/i386-linux-gnu/libusb-1.0.so 
/usr/lib/i386-linux-gnu/libpthread.so 
/opt/ros/fuerte/lib/libopencv_calib3d.so 
/opt/ros/fuerte/lib/libopencv_contrib.so 
/opt/ros/fuerte/lib/libopencv_core.so 
/opt/ros/fuerte/lib/libopencv_features2d.so 
/opt/ros/fuerte/lib/libopencv_flann.so 
/opt/ros/fuerte/lib/libopencv_gpu.so 
/opt/ros/fuerte/lib/libopencv_highgui.so 
/opt/ros/fuerte/lib/libopencv_imgproc.so 
/opt/ros/fuerte/lib/libopencv_legacy.so 
/opt/ros/fuerte/lib/libopencv_ml.so 
/opt/ros/fuerte/lib/libopencv_nonfree.so 
/opt/ros/fuerte/lib/libopencv_objdetect.so 
/opt/ros/fuerte/lib/libopencv_photo.so 
/opt/ros/fuerte/lib/libopencv_stitching.so 
/opt/ros/fuerte/lib/libopencv_ts.so 
/opt/ros/fuerte/lib/libopencv_video.so 
/opt/ros/fuerte/lib/libopencv_videostab.so 
/opt/ros/fuerte/lib/libopencv_calib3d.so 
/opt/ros/fuerte/lib/libopencv_contrib.so 
/opt/ros/fuerte/lib/libopencv_core.so 
/opt/ros/fuerte/lib/libopencv_features2d.so 
/opt/ros/fuerte/lib/libopencv_flann.so 
/opt/ros/fuerte/lib/libopencv_gpu.so 
/opt/ros/fuerte/lib/libopencv_highgui.so 
/opt/ros/fuerte/lib/libopencv_imgproc.so 
/opt/ros/fuerte/lib/libopencv_legacy.so 
/opt/ros/fuerte/lib/libopencv_ml.so 
/opt/ros/fuerte/lib/libopencv_nonfree.so 
/opt/ros/fuerte/lib/libopencv_objdetect.so 
/opt/ros/fuerte/lib/libopencv_photo.so 
/opt/ros/fuerte/lib/libopencv_stitching.so 
/opt/ros/fuerte/lib/libopencv_ts.so 
/opt/ros/fuerte/lib/libopencv_video.so 
/opt/ros/fuerte/lib/libopencv_videostab.so 
/usr/lib/i386-linux-gnu/libz.so 
/usr/lib/i386-linux-gnu/libjpeg.so 
/usr/lib/i386-linux-gnu/libpng.so 
/usr/lib/i386-linux-gnu/libz.so 
/usr/lib/libswscale.so 
/usr/lib/libavformat.so 
/usr/lib/libavcodec.so 
/usr/lib/libavutil.so 
/usr/lib/i386-linux-gnu/libz.so 
/usr/lib/libbz2.so 
/usr/lib/libdc1394.so 
/usr/lib/i386-linux-gnu/libv4l2.so 
/usr/lib/i386-linux-gnu/libv4lconvert.so
)

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
