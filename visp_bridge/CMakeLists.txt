cmake_minimum_required(VERSION 2.8.3)
project(visp_bridge)

find_package(Boost REQUIRED)

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
)

include_directories(include)

include_directories(SYSTEM
  ${VISP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
link_directories(
  ${VISP_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

# Add package definitions
add_definitions(${VISP_DEFINITIONS})

## Declare a cpp library
add_library(visp_bridge
  src/conversions/3dpose.cpp
  src/conversions/image.cpp
  src/conversions/camera.cpp
  src/compat/vpQuaternionVector.cpp
)
target_link_libraries(visp_bridge ${VISP_LIBRARIES})

add_executable(camera_tool src/conversions/camera_tool.cpp)
target_link_libraries(camera_tool ${VISP_LIBRARIES} ${catkin_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY} visp_bridge)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} camera_tool
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

