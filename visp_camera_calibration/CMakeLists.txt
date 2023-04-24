cmake_minimum_required(VERSION 3.5)
project(visp_camera_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(visp_bridge REQUIRED)
find_package(VISP REQUIRED COMPONENTS visp_core visp_blob visp_vision)

# messages and services interface generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CalibPointArray.msg"
  "msg/CalibPoint.msg"
  "msg/ImageAndPoints.msg"
  "msg/ImagePoint.msg"
  "srv/Calibrate.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

add_library(${PROJECT_NAME}_common
  src/names.cpp
)

ament_target_dependencies(${PROJECT_NAME}_common
  rclcpp
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

target_include_directories(${PROJECT_NAME}_common PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

add_executable(${PROJECT_NAME}_calibrator
  src/calibrator.cpp
  src/calibrator_main.cpp)

target_link_libraries(${PROJECT_NAME}_calibrator
  ${PROJECT_NAME}_common
  "${cpp_typesupport_target}"
  ${VISP_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_calibrator
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visp_bridge
)

# target image_processing
add_executable(${PROJECT_NAME}_image_processing
  src/image_processing.cpp
  src/image_processing_main.cpp)

target_link_libraries(${PROJECT_NAME}_image_processing
  ${PROJECT_NAME}_common
  "${cpp_typesupport_target}"
  ${VISP_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_image_processing
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visp_bridge
  camera_calibration_parsers
)

# target camera
add_executable(${PROJECT_NAME}_camera
  src/camera.cpp
  src/camera_main.cpp)

target_link_libraries(${PROJECT_NAME}_camera
  ${PROJECT_NAME}_common
  "${cpp_typesupport_target}"
  ${VISP_LIBRARIES})

ament_target_dependencies(${PROJECT_NAME}_camera
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visp_bridge
  camera_calibration_parsers
)

install(
  TARGETS
  ${PROJECT_NAME}_calibrator
  ${PROJECT_NAME}_image_processing
  ${PROJECT_NAME}_camera
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
