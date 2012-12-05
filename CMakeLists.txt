cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

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

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(visp_camera_calibration_common src/names.cpp)




rosbuild_add_executable(visp_camera_calibration_calibrator src/calibrator.cpp src/calibrator_main.cpp)
target_link_libraries(visp_camera_calibration_calibrator visp_camera_calibration_common visp_bridge visp-2)

rosbuild_add_executable(visp_camera_calibration_image_processing src/image_processing.cpp src/image_processing_main.cpp)
target_link_libraries(visp_camera_calibration_image_processing visp_camera_calibration_common visp_bridge visp-2)


rosbuild_add_executable(visp_camera_calibration_camera src/camera.cpp src/camera_main.cpp)
target_link_libraries(visp_camera_calibration_camera visp_camera_calibration_common visp_bridge visp-2)
