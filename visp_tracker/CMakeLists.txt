cmake_minimum_required(VERSION 3.0.0)
project(visp_tracker)

find_package(Boost REQUIRED COMPONENTS filesystem thread)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  image_proc
  image_transport
  message_generation
  nodelet
  resource_retriever
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  )

# ViSP cannot be found by Catkin.
# see https://github.com/ros/catkin/issues/606
find_package(VISP REQUIRED)


# Generate messages and services.
add_message_files(
  DIRECTORY 
    msg
  FILES
    KltPoint.msg
    KltPoints.msg
    KltSettings.msg
    MovingEdgeSettings.msg
    MovingEdgeSite.msg
    MovingEdgeSites.msg
    TrackerSettings.msg
  )
  
add_service_files(
  DIRECTORY 
    srv
  FILES
    Init.srv
  )
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

# Dynamic reconfigure.
generate_dynamic_reconfigure_options(
    cfg/ModelBasedSettings.cfg
    cfg/ModelBasedSettingsEdge.cfg
    cfg/ModelBasedSettingsKlt.cfg)


catkin_package(
  LIBRARIES
    trackerNodelet
    ${PROJECT_NAME} 

  CATKIN_DEPENDS
    dynamic_reconfigure
    geometry_msgs
    message_runtime
    nodelet
    roscpp
    sensor_msgs
    std_msgs
    tf

  DEPENDS
    VISP
  )

include_directories(SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )

include_directories(
  "${CMAKE_CURRENT_SOURCE_DIR}/src/lib${PROJECT_NAME}")

# Make sure Boost.Filesystem v3 is used.
add_definitions(-DBOOST_FILESYSTEM_VERSION=3)

#############
# Libraries #
#############

# Library gathering libvisp_tracker used by all nodes.
add_library(${PROJECT_NAME}
  src/lib${PROJECT_NAME}/callbacks.cpp
  src/lib${PROJECT_NAME}/callbacks.hh
  src/lib${PROJECT_NAME}/conversion.cpp
  src/lib${PROJECT_NAME}/conversion.hh
  src/lib${PROJECT_NAME}/file.cpp
  src/lib${PROJECT_NAME}/file.hh
  src/lib${PROJECT_NAME}/names.cpp
  src/lib${PROJECT_NAME}/names.hh
  src/lib${PROJECT_NAME}/tracker-client.cpp
  src/lib${PROJECT_NAME}/tracker-client.hh
  src/lib${PROJECT_NAME}/tracker-viewer.cpp
  src/lib${PROJECT_NAME}/tracker-viewer.hh
  src/lib${PROJECT_NAME}/tracker.cpp
  src/lib${PROJECT_NAME}/tracker.hh)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${VISP_LIBRARIES}
  )
target_link_libraries(${PROJECT_NAME}
  ${dynamic_reconfigure_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${image_proc_LIBRARIES}
  ${image_transport_LIBRARIES}
  ${message_generation_LIBRARIES}
  ${nodelet_LIBRARIES}
  ${resource_retriever_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${tf_LIBRARIES}
  )

if(CATKIN_ENABLE_TESTING)
  add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_bag)
else()
  add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
endif()
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(DIRECTORY src/lib${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
#install(DIRECTORY cfg/cpp/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)

############
# Nodelets #
############

# Tracker, client, viewer nodelets.
add_library(trackerNodelet
  src/nodelets/client.cpp
  src/nodelets/tracker.cpp
  src/nodelets/viewer.cpp)

target_link_libraries(trackerNodelet ${PROJECT_NAME})
target_link_libraries(trackerNodelet
  ${roscpp_LIBRARIES}
  ${nodelet_LIBRARIES}
  ${VISP_LIBRARIES}
  )

install(TARGETS trackerNodelet ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


########
# Node #
########

add_executable(tracker src/nodes/tracker.cpp)
target_link_libraries(tracker
  ${roscpp_LIBRARIES}
  ${nodelet_LIBRARIES})

add_executable(${PROJECT_NAME}_client src/nodes/client.cpp)
target_link_libraries(${PROJECT_NAME}_client
  ${roscpp_LIBRARIES}
  ${nodelet_LIBRARIES})

add_executable(${PROJECT_NAME}_viewer src/nodes/viewer.cpp)
target_link_libraries(${PROJECT_NAME}_viewer
  ${roscpp_LIBRARIES}
  ${nodelet_LIBRARIES})

add_dependencies(tracker ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_client ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(TARGETS tracker ${PROJECT_NAME}_client ${PROJECT_NAME}_viewer
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/tf_localization.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


#############
# Bag files #
#############

# Tutorial
if(CATKIN_ENABLE_TESTING)
  catkin_download_test_data(
    ${PROJECT_NAME}_bag
    https://github.com/lagadic/vision_visp/releases/download/vision_visp-0.5.0/tutorial-static-box.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bag
    MD5 1578dedd48d3f9f5515a8737845ae882)

  install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bag
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endif()

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
