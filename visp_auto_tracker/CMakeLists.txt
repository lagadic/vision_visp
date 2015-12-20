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

catkin_package(
  LIBRARIES 
    auto_tracker

  CATKIN_DEPENDS
    geometry_msgs
    message_filters
    roscpp
    sensor_msgs
    std_msgs

  DEPENDS
    VISP
  )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/flashcode_mbt)

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

if(VISP_VERSION VERSION_LESS 2.10.0)
  add_library(visp_auto_tracker_qrcode_detector
    flashcode_mbt/detectors/detector_base.cpp
    flashcode_mbt/detectors/detector_base.h
    flashcode_mbt/detectors/qrcode/detector.cpp
    flashcode_mbt/detectors/qrcode/detector.h
  )
  target_link_libraries(visp_auto_tracker_qrcode_detector
    ${resource_retriever_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    zbar)

  add_library(visp_auto_tracker_datamatrix_detector
    flashcode_mbt/detectors/datamatrix/detector.cpp
    flashcode_mbt/detectors/datamatrix/detector.h
    flashcode_mbt/detectors/detector_base.cpp
    flashcode_mbt/detectors/detector_base.h
  )
  target_link_libraries(visp_auto_tracker_datamatrix_detector
    ${resource_retriever_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    dmtx)
endif()

add_library(auto_tracker
  flashcode_mbt/libauto_tracker/events.h
  flashcode_mbt/libauto_tracker/logfilewriter.hpp
  flashcode_mbt/libauto_tracker/states.hpp
  flashcode_mbt/libauto_tracker/threading.cpp
  flashcode_mbt/libauto_tracker/threading.h
  flashcode_mbt/libauto_tracker/tracking.cpp
  flashcode_mbt/libauto_tracker/tracking.h)

if(VISP_VERSION VERSION_LESS 2.10.0)
  target_link_libraries(auto_tracker
      visp_auto_tracker_cmd_line
      visp_auto_tracker_qrcode_detector
      visp_auto_tracker_datamatrix_detector
      ${resource_retriever_LIBRARIES}
      ${catkin_LIBRARIES}
      ${Boost_LIBRARIES}
  )
else()
  target_link_libraries(auto_tracker
      visp_auto_tracker_cmd_line
      ${resource_retriever_LIBRARIES}
      ${catkin_LIBRARIES}
      ${Boost_LIBRARIES}
  )
endif()

add_executable(visp_auto_tracker
  src/main.cpp
  src/names.cpp
  src/names.h
  src/node.cpp
  src/node.h
  )

add_dependencies(visp_auto_tracker visp_auto_tracker_bag)

if(VISP_VERSION VERSION_LESS 2.10.0)
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
      auto_tracker
      visp_auto_tracker
      visp_auto_tracker_cmd_line
      visp_auto_tracker_datamatrix_detector
      visp_auto_tracker_qrcode_detector
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  target_link_libraries(visp_auto_tracker
    auto_tracker
    ${resource_retriever_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES})

  install(
    TARGETS
      auto_tracker
      visp_auto_tracker
      visp_auto_tracker_cmd_line
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()

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
if(CATKIN_ENABLE_TESTING)
  catkin_download_test_data(
    visp_auto_tracker_bag
    https://github.com/lagadic/vision_visp/releases/download/vision_visp-0.5.0/tutorial-qrcode.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bag
    MD5 0f80ceea2610b8400591ca7aff764dfa)

  install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bag
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endif()
