cmake_minimum_required(VERSION 3.5)
project(visp_auto_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(Boost_INCLUDE_DIR /usr/include/boost)
set(BOOST_LIBRARYDIR /usr/lib/x86_64-linux-gnu/)

# ignore BOOST deprecated headers
add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")
add_definitions("-DBOOST_BIND_GLOBAL_PLACEHOLDERS")

find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visp_bridge REQUIRED)
find_package(visp_tracker REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(VISP REQUIRED COMPONENTS visp_core)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# ############
# Libraries #
# ############
# Library gathering libauto_tracker used by all nodes.
add_library(${PROJECT_NAME}_common
  src/cmd_line.cpp
  src/tracking.cpp
)

ament_target_dependencies(${PROJECT_NAME}_common
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  resource_retriever
  visp_tracker
  visp_bridge
  VISP
)

target_include_directories(${PROJECT_NAME}_common PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${PROJECT_NAME}_common
  ${VISP_LIBRARIES}

  # "${cpp_typesupport_target}"
)

add_executable(${PROJECT_NAME}_main
  src/main.cpp
  src/names.cpp
  src/autotracker.cpp
)
target_link_libraries(${PROJECT_NAME}_main
  "${cpp_typesupport_target}"
  ${VISP_LIBRARIES}
  ${PROJECT_NAME}_common
)

ament_target_dependencies(${PROJECT_NAME}_main
  rclcpp
  message_filters
  std_msgs
  image_transport
  geometry_msgs
  resource_retriever
  sensor_msgs
  visp_tracker
  visp_bridge
)

target_include_directories(${PROJECT_NAME}_main PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

if(EXTRA_CXX_FLAGS)
  set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS ${EXTRA_CXX_FLAGS})
  set_target_properties(${PROJECT_NAME}_cmd_line PROPERTIES COMPILE_FLAGS ${EXTRA_CXX_FLAGS})
  set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS ${EXTRA_CXX_FLAGS})
endif()

install(
  TARGETS
  ${PROJECT_NAME}_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch models
  DESTINATION share/${PROJECT_NAME}
)

# ############
# Bag files #
# ############
include(ExternalProject)
ExternalProject_Add(
  external_bag
  PREFIX "externals"
  URL https://github.com/lagadic/vision_visp/releases/download/vision_visp-0.5.0/tutorial-qrcode.bag
  DOWNLOAD_NO_EXTRACT true
  CONFIGURE_COMMAND ""
  PATCH_COMMAND rosbags-convert ../tutorial-qrcode.bag
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)

ExternalProject_Get_Property(external_bag DOWNLOADED_FILE)
if(EXISTS ${DOWNLOADED_FILE})
  message("Successfully download ${DOWNLOADED_FILE}")
endif()

install(DIRECTORY
  ${CMAKE_CURRENT_BINARY_DIR}/externals/src/tutorial-qrcode
  DESTINATION share/bag
)
message("Bagfile installed in ${CMAKE_INSTALL_PREFIX}/share/bag/tutorial-qrcode")

ament_export_dependencies(rosidl_default_runtime)

ament_package()
