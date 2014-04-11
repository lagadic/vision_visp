cmake_minimum_required(VERSION 2.8.3)
project(visp_auto_tracker)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  resource_retriever
  roscpp
  sensor_msgs
  std_msgs
  visp_bridge
  visp_tracker
)
# ViSP cannot be found by Catkin.
# see https://github.com/ros/catkin/issues/606
find_package(VISP)

find_package(Boost REQUIRED
  COMPONENTS filesystem system signals regex date_time program_options thread)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
link_directories(
  ${BULLET_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
  ${VISP_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
  )

catkin_package(
   LIBRARIES auto_tracker

   CATKIN_DEPENDS
   geometry_msgs
   message_filters
   roscpp
   sensor_msgs
   std_msgs

   DEPENDS
   VISP
   )

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/flashcode_mbt
  ${CMAKE_CURRENT_SOURCE_DIR}/flashcode_mbt/detectors
  )

# Add ViSP definitions.
add_definitions(${VISP_DEFINITIONS})

# Library gathering libauto_tracker used by all nodes.
add_library(visp_auto_tracker_cmd_line
  flashcode_mbt/cmd_line/cmd_line.cpp
  flashcode_mbt/cmd_line/cmd_line.h
)
target_link_libraries(visp_auto_tracker_cmd_line
  ${resource_retriever_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES})

add_library(visp_auto_tracker_qrcode_detector
  flashcode_mbt/detectors/detector_base.h
  flashcode_mbt/detectors/detector_base.cpp
  flashcode_mbt/detectors/qrcode/detector.h
  flashcode_mbt/detectors/qrcode/detector.cpp
)
target_link_libraries(visp_auto_tracker_qrcode_detector
  ${resource_retriever_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  dmtx zbar)

add_library(auto_tracker
  flashcode_mbt/libauto_tracker/states.hpp
  flashcode_mbt/libauto_tracker/events.h
  flashcode_mbt/libauto_tracker/tracking.h
  flashcode_mbt/libauto_tracker/tracking.cpp
  flashcode_mbt/libauto_tracker/logfilewriter.hpp
  flashcode_mbt/libauto_tracker/threading.h
  flashcode_mbt/libauto_tracker/threading.cpp)
target_link_libraries(auto_tracker
  visp_auto_tracker_cmd_line
  visp_auto_tracker_qrcode_detector
  ${resource_retriever_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )



add_library(visp_auto_tracker_datamatrix_detector
  flashcode_mbt/detectors/detector_base.h
  flashcode_mbt/detectors/detector_base.cpp
  flashcode_mbt/detectors/datamatrix/detector.h
  flashcode_mbt/detectors/datamatrix/detector.cpp
)
target_link_libraries(visp_auto_tracker_datamatrix_detector
  ${resource_retriever_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  dmtx)


add_executable(visp_auto_tracker
  src/main.cpp
  src/node.cpp
  src/node.h
  src/names.cpp
  src/names.h
  )

add_dependencies(visp_auto_tracker visp_auto_tracker_bag)

target_link_libraries(visp_auto_tracker
  auto_tracker
  visp_auto_tracker_qrcode_detector
  visp_auto_tracker_datamatrix_detector
  visp_auto_tracker_cmd_line
  dmtx zbar)

target_link_libraries(visp_auto_tracker
  ${resource_retriever_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES})

install(
  TARGETS
    visp_auto_tracker
    auto_tracker
    visp_auto_tracker_qrcode_detector
    visp_auto_tracker_datamatrix_detector
    visp_auto_tracker_cmd_line
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
# Bag files #
#############

# Tutorial
catkin_download_test_data(
  visp_auto_tracker_bag
  https://github.com/downloads/lagadic/visp_auto_tracker/tutorial-qrcode.bag
  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bag
  MD5 0f80ceea2610b8400591ca7aff764dfa)

install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bag
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

