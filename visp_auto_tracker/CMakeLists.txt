cmake_minimum_required(VERSION 3.0.0)
project(visp_auto_tracker)

# Since visp 3.5.0 to avoid the following warning that makes build unstable
#
# /usr/include/OGRE/OgreMemorySTLAllocator.h:130:23: warning: ISO C++17 does not allow
#   ‘register’ storage class specifier [-Wregister]
#
# we try to add -Wno-register option
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-Wno-register" COMPILER_SUPPORTS_W_NO_REGISTER)

#catkin_lint: ignore endblock_args indentation
if(COMPILER_SUPPORTS_W_NO_REGISTER)
  set(EXTRA_CXX_FLAGS "-Wno-register")
endif()

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

if(VISP_HAVE_ZBAR OR VISP_HAVE_DMTX OR VISP_HAVE_APRILTAG)

# Boost signals component is no longer being actively maintained since boost 1.63.0
# Keept here as optional for compat with older ROS versions, but for Neotic this component is useless.
find_package(Boost REQUIRED
  COMPONENTS filesystem system signals regex date_time program_options thread)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES auto_tracker

  CATKIN_DEPENDS
    geometry_msgs
    message_filters
    roscpp
    sensor_msgs
    std_msgs
    visp_tracker

  DEPENDS
    VISP
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/flashcode_mbt)

# Library gathering libauto_tracker used by all nodes.
add_library(${PROJECT_NAME}_cmd_line
  flashcode_mbt/cmd_line/cmd_line.cpp
  flashcode_mbt/cmd_line/cmd_line.h
)
target_link_libraries(${PROJECT_NAME}_cmd_line
  ${resource_retriever_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES})

#catkin_lint: ignore unquoted_string_op
if(VISP_VERSION VERSION_LESS "2.10.0")
  add_library(${PROJECT_NAME}_qrcode_detector
    flashcode_mbt/detectors/detector_base.cpp
    flashcode_mbt/detectors/detector_base.h
    flashcode_mbt/detectors/qrcode/detector.cpp
    flashcode_mbt/detectors/qrcode/detector.h
  )
  target_link_libraries(${PROJECT_NAME}_qrcode_detector
    ${resource_retriever_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    zbar)

  add_library(${PROJECT_NAME}_datamatrix_detector
    flashcode_mbt/detectors/datamatrix/detector.cpp
    flashcode_mbt/detectors/datamatrix/detector.h
    flashcode_mbt/detectors/detector_base.cpp
    flashcode_mbt/detectors/detector_base.h
  )
  target_link_libraries(${PROJECT_NAME}_datamatrix_detector
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

#catkin_lint: ignore unquoted_string_op
if(VISP_VERSION VERSION_LESS "2.10.0")
  target_link_libraries(auto_tracker
      ${PROJECT_NAME}_cmd_line
      ${PROJECT_NAME}_qrcode_detector
      ${PROJECT_NAME}_datamatrix_detector
      ${resource_retriever_LIBRARIES}
      ${catkin_LIBRARIES}
      ${Boost_LIBRARIES}
  )
else()
  target_link_libraries(auto_tracker
      ${PROJECT_NAME}_cmd_line
      ${resource_retriever_LIBRARIES}
      ${catkin_LIBRARIES}
      ${Boost_LIBRARIES}
  )
endif()

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/names.cpp
  src/names.h
  src/node.cpp
  src/node.h
  )

if(EXTRA_CXX_FLAGS)
    set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS ${EXTRA_CXX_FLAGS})
    set_target_properties(${PROJECT_NAME}_cmd_line PROPERTIES COMPILE_FLAGS ${EXTRA_CXX_FLAGS})
    set_target_properties(auto_tracker PROPERTIES COMPILE_FLAGS ${EXTRA_CXX_FLAGS})
endif()

if(CATKIN_ENABLE_TESTING)
  add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_bag)
endif()

#catkin_lint: ignore unquoted_string_op
if(VISP_VERSION VERSION_LESS "2.10.0")
  target_link_libraries(${PROJECT_NAME}
    auto_tracker
    ${PROJECT_NAME}_qrcode_detector
    ${PROJECT_NAME}_datamatrix_detector
    ${PROJECT_NAME}_cmd_line
    dmtx zbar)

  target_link_libraries(${PROJECT_NAME}
    ${resource_retriever_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES})

  install(
    TARGETS
      auto_tracker
      ${PROJECT_NAME}
      ${PROJECT_NAME}_cmd_line
      ${PROJECT_NAME}_datamatrix_detector
      ${PROJECT_NAME}_qrcode_detector
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  target_link_libraries(${PROJECT_NAME}
    auto_tracker
    ${resource_retriever_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES})

  install(
    TARGETS
      auto_tracker
      ${PROJECT_NAME}
      ${PROJECT_NAME}_cmd_line
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
    ${PROJECT_NAME}_bag
    https://github.com/lagadic/vision_visp/releases/download/vision_visp-0.5.0/tutorial-qrcode.bag
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bag
    MD5 0f80ceea2610b8400591ca7aff764dfa)

  install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/bag
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endif()

else() # VISP_HAVE_ZBAR OR VISP_HAVE_DMTX OR VISP_HAVE_APRILTAG
message("${PROJECT_NAME} build is disabled since ViSP is not built with libzbar-dev 3rd party")
endif() # VISP_HAVE_ZBAR OR VISP_HAVE_DMTX OR VISP_HAVE_APRILTAG

