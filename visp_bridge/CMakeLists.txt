cmake_minimum_required(VERSION 3.5)
project(visp_bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(VISP REQUIRED COMPONENTS visp_core)

# add library 
add_library(${PROJECT_NAME} src/3dpose.cpp src/camera.cpp src/image.cpp src/path_retriever.cpp)

ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs
  rclcpp
  sensor_msgs
  VISP
)

include(GenerateExportHeader)
generate_export_header(${PROJECT_NAME} EXPORT_FILE_NAME ${PROJECT_NAME}/${PROJECT_NAME}_export.h)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${VISP_INCLUDE_DIRS})

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/${PROJECT_NAME}_export.h
  DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME})

# tools
add_executable(${PROJECT_NAME}_cam_param_converter tool/cam_param_converter.cpp)

target_include_directories(${PROJECT_NAME}_cam_param_converter PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${VISP_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}_cam_param_converter PUBLIC
  ${camera_calibration_parsers_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${VISP_LIBRARIES}
  ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}_cam_param_converter PRIVATE
  rcpputils::rcpputils)

install(TARGETS ${PROJECT_NAME}_cam_param_converter
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  ${CMAKE_SOURCE_DIR}/data
  DESTINATION share
)

# visp_bridge_lib_dir is passed as APPEND_LIBRARY_DIRS for each ament_add_gtest call so
# the project library that they link against is on the library path.
# This is especially important on Windows.
# This is overwritten each loop, but which one it points to doesn't really matter.
set(visp_bridge_lib_dir "$<TARGET_FILE_DIR:${PROJECT_NAME}>")

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(VISP)

# Mark cpp header files for installation
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_package()
