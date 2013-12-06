cmake_minimum_required(VERSION 2.8.3)
project(visp_bridge)

# By default set RelWithDebInfo configuration
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE String "Choose the type of build, options are: None Debug Release" FORCE)
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
)

find_package(VISP REQUIRED)

###################################
## catkin specific configuration ##
###################################
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

  DEPENDS 
    VISP  
)

###########
## Build ##
###########
include_directories(
  include
  ${VISP_INCLUDE_DIRS}
)

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


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
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

