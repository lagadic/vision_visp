cmake_minimum_required(VERSION 2.8.3)
project(visp_tracker)

# By default set RelWithDebInfo configuration
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE String "Choose the type of build, options are: None Debug Release" FORCE)
endif()

find_package(catkin REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem thread)
find_package(VISP REQUIRED)

# Look for Bullet
find_package(PkgConfig REQUIRED)
pkg_check_modules(BULLET bullet)
include_directories(${BULLET_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  image_proc
  image_transport
  message_generation
  nodelet
  resource_retriever
  roscpp
  sensor_msgs
  std_msgs
  tf
  )

catkin_python_setup()

include_directories(SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )
link_directories(
  ${catkin_LIBRARY_DIRS}
  ${VISP_LIBRARY_DIRS}
  )

include_directories(
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/libvisp_tracker"
  "${CMAKE_CURRENT_SOURCE_DIR}/cfg/cpp")

# Generate messages and services.
add_message_files(
  DIRECTORY msg
  FILES
  KltPoint.msg
  KltPoints.msg
  KltSettings.msg
  MovingEdgeSettings.msg
  MovingEdgeSite.msg
  MovingEdgeSites.msg
  )
add_service_files(
  DIRECTORY srv
  FILES
  Init.srv
  )
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

# Dynamic reconfigure.
generate_dynamic_reconfigure_options(cfg/ModelBasedSettings.cfg)

catkin_package(
   LIBRARIES visp_tracker trackerNodelet

   CATKIN_DEPENDS
   geometry_msgs
   message_runtime
   roscpp
   nodelet
   sensor_msgs
   std_msgs
   dynamic_reconfigure

   DEPENDS
#   VISP
   )

# Add package definitions
add_definitions(${VISP_DEFINITIONS})

# Make sure Boost.Filesystem v2 is used.
add_definitions(-DBOOST_FILESYSTEM_VERSION=2)

#############
# Libraries #
#############

# Library gathering libvisp_tracker used by all nodes.
add_library(visp_tracker
  src/libvisp_tracker/callbacks.cpp
  src/libvisp_tracker/callbacks.hh
  src/libvisp_tracker/conversion.cpp
  src/libvisp_tracker/conversion.hh
  src/libvisp_tracker/file.cpp
  src/libvisp_tracker/file.hh
  src/libvisp_tracker/names.cpp
  src/libvisp_tracker/names.hh
  src/libvisp_tracker/tracker.cpp
  src/libvisp_tracker/tracker.hh
  src/libvisp_tracker/tracker-client.cpp
  src/libvisp_tracker/tracker-client.hh
  src/libvisp_tracker/tracker-viewer.cpp
  src/libvisp_tracker/tracker-viewer.hh)

target_link_libraries(visp_tracker
  ${Boost_LIBRARIES} 
#${VISP_LIBRARIES}
  )
target_link_libraries(visp_tracker
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

add_dependencies(visp_tracker ${PROJECT_NAME}_gencpp visp_tracker_bag)
add_dependencies(visp_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(DIRECTORY src/lib${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
install(DIRECTORY cfg/cpp/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

############
# Nodelets #
############

# Tracker, client, viewer nodelets.
add_library(trackerNodelet
  src/nodelets/tracker.cpp
  src/nodelets/client.cpp
  src/nodelets/viewer.cpp)

target_link_libraries(trackerNodelet visp_tracker)
target_link_libraries(trackerNodelet
  ${roscpp_LIBRARIES}
  ${nodelet_LIBRARIES}
  ${VISP_LIBRARIES}
  )

install(TARGETS visp_tracker trackerNodelet
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

add_executable(client src/nodes/client.cpp)
target_link_libraries(client
  ${roscpp_LIBRARIES}
  ${nodelet_LIBRARIES})

add_executable(viewer src/nodes/viewer.cpp)
target_link_libraries(viewer
  ${roscpp_LIBRARIES}
  ${nodelet_LIBRARIES})

add_dependencies(tracker ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(client ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(TARGETS tracker client viewer
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
catkin_download_test_data(
  visp_tracker_bag
  https://github.com/downloads/lagadic/visp_tracker/tutorial-static-box.bag
  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bag
  MD5 1578dedd48d3f9f5515a8737845ae882)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bag
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
