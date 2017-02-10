cmake_minimum_required(VERSION 2.8.3)
project(visp_bridge)

find_package(Boost REQUIRED COMPONENTS filesystem program_options)

find_package(catkin REQUIRED COMPONENTS
  camera_calibration_parsers
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
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
    camera_calibration_parsers
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs

  DEPENDS
    Boost
    VISP
)

include_directories(include)

include_directories(SYSTEM
  ${VISP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Make sure Boost.Filesystem v3 is used.
add_definitions(-DBOOST_FILESYSTEM_VERSION=3)

## Declare a cpp library
add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/3dpose.h
  include/${PROJECT_NAME}/camera.h
  include/${PROJECT_NAME}/image.h
  include/${PROJECT_NAME}/vpQuaternionVector.h
  src/compat/vpQuaternionVector.cpp
  src/conversions/3dpose.cpp
  src/conversions/camera.cpp
  src/conversions/image.cpp
)
target_link_libraries(${PROJECT_NAME} ${VISP_LIBRARIES})

add_executable(${PROJECT_NAME}_convert_camera_parameters src/conversions/convert_cam_param_file.cpp)
target_link_libraries(${PROJECT_NAME}_convert_camera_parameters ${VISP_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_NAME})

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_convert_camera_parameters 
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

