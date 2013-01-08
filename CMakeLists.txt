cmake_minimum_required(VERSION 2.8.3)
project(visp_tracker)

find_package(catkin REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem thread)
find_package(VISP REQUIRED)
#FIXME: look for bullet.
include_directories(SYSTEM /opt/ros/groovy/stacks/bullet/include)

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
  ${catkin_LIBRARY_DIRS}
  )

include_directories(
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/libvisp_tracker"
  "${CMAKE_CURRENT_SOURCE_DIR}/cfg/cpp")

# Generate messages and services.
add_message_files(
  DIRECTORY msg
  FILES
  MovingEdge.msg
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
generate_dynamic_reconfigure_options(cfg/MovingEdge.cfg)

catkin_package(
   LIBRARIES visp_tracker trackerNodelet

   CATKIN_DEPENDS
   geometry_msgs
   message_generation
   roscpp
   nodelet
   sensor_msgs
   std_msgs

   DEPENDS
   visp
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
  src/libvisp_tracker/conversion.cpp
  src/libvisp_tracker/callbacks.cpp
  src/libvisp_tracker/file.cpp
  src/libvisp_tracker/tracker.cpp
  src/libvisp_tracker/tracker-client.cpp
  src/libvisp_tracker/tracker-viewer.cpp
  src/libvisp_tracker/names.cpp)
target_link_libraries(visp_tracker
  ${Boost_LIBRARIES} ${VISP_LIBRARIES}
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

install(TARGETS tracker client viewer
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
# Bag files #
#############

# Tutorial
download_test_data(
  https://github.com/downloads/laas/visp_tracker/tutorial-static-box.bag
  bag/tutorial-static-box.bag
  1578dedd48d3f9f5515a8737845ae882)
