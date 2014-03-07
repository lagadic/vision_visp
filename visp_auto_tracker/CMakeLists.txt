cmake_minimum_required(VERSION 2.8.3)
project(visp_auto_tracker)

# By default set RelWithDebInfo configuration
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE String "Choose the type of build, options are: None Debug Release" FORCE)
endif()

find_package(catkin REQUIRED COMPONENTS
  visp_bridge
  visp_tracker
  resource_retriever
  message_filters
)

# TO FIX, VISP_DEFINITIONS and VISP_LIBRARY_DIRS should come from visp_bridge dependency
find_package(VISP REQUIRED)
add_definitions(${VISP_DEFINITIONS})
link_directories(${VISP_LIBRARY_DIRS})

find_package(Boost REQUIRED COMPONENTS filesystem system signals regex date_time program_options thread)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

catkin_package()


include_directories("flashcode_mbt")
include_directories("flashcode_mbt/detectors")

# Library gathering libvisp_tracker used by all nodes.
add_library(auto_tracker
  flashcode_mbt/libauto_tracker/states.hpp
  flashcode_mbt/libauto_tracker/events.h
  flashcode_mbt/libauto_tracker/tracking.h
  flashcode_mbt/libauto_tracker/tracking.cpp
  flashcode_mbt/libauto_tracker/logfilewriter.hpp
  flashcode_mbt/libauto_tracker/threading.h
  flashcode_mbt/libauto_tracker/threading.cpp)

add_library(cmd_line
  flashcode_mbt/cmd_line/cmd_line.cpp
  flashcode_mbt/cmd_line/cmd_line.h
)

add_subdirectory(flashcode_mbt/detectors)

add_executable(visp_auto_tracker
  src/main.cpp
  src/node.cpp
  src/node.h
  src/names.cpp
  src/names.h
  )

#rosbuild_link_boost(visp_auto_tracker filesystem system signals regex date_time program_options thread)

add_dependencies(visp_auto_tracker visp_auto_tracker_bag)
#target_link_libraries(visp_auto_tracker auto_tracker qrcode_detector datamatrix_detector cmd_line dmtx zbar )
target_link_libraries(visp_auto_tracker auto_tracker qrcode_detector datamatrix_detector cmd_line dmtx zbar)
target_link_libraries(visp_auto_tracker ${resource_retriever_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(
  TARGETS
    visp_auto_tracker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
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

