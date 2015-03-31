cmake_minimum_required(VERSION 2.8.3)
project(visp_bridge)

find_package(Boost REQUIRED COMPONENTS filesystem program_options)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  camera_calibration_parsers
)

# ViSP cannot be found by Catkin.
# see https://github.com/ros/catkin/issues/606
find_package(VISP REQUIRED)


catkin_package(
  INCLUDE_DIRS
    include

  LIBRARIES
    ${PROJECT_NAME}

  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs
    camera_calibration_parsers

  DEPENDS
    VISP
    Boost
)

include_directories(include)

include_directories(SYSTEM
  ${VISP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)
#link_directories(
#  ${VISP_LIBRARY_DIRS}
#  ${catkin_LIBRARY_DIRS}
#  ${Boost_LIBRARY_DIRS}
#)

# Make sure Boost.Filesystem v3 is used.
add_definitions(-DBOOST_FILESYSTEM_VERSION=3)

# Add package definitions
add_definitions(${VISP_DEFINITIONS})

## Declare a cpp library
add_library(visp_bridge
  include/visp_bridge/3dpose.h
  include/visp_bridge/image.h
  include/visp_bridge/camera.h
  include/visp_bridge/vpQuaternionVector.h
  src/conversions/3dpose.cpp
  src/conversions/image.cpp
  src/conversions/camera.cpp
  src/compat/vpQuaternionVector.cpp
)
target_link_libraries(visp_bridge ${VISP_LIBRARIES})

add_executable(convert_cam_param_file src/conversions/convert_cam_param_file.cpp)
target_link_libraries(convert_cam_param_file ${VISP_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} visp_bridge)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} convert_cam_param_file
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

