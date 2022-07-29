cmake_minimum_required(VERSION 3.5)
project(visp_hand2eye_calibration)

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
find_package(visp_bridge REQUIRED)
find_package(VISP REQUIRED COMPONENTS visp_core visp_vision)

# include_directories(include)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TransformArray.msg"
  "srv/ComputeEffectorCamera.srv"
  "srv/ComputeEffectorCameraQuick.srv"
  "srv/Reset.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

add_library(${PROJECT_NAME}_common
  src/names.cpp
)
ament_target_dependencies(${PROJECT_NAME}_common
  sensor_msgs geometry_msgs rclcpp
)
target_include_directories(${PROJECT_NAME}_common PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(${PROJECT_NAME}_calibrator
  src/calibrator.cpp
  src/calibrator_main.cpp
)

target_link_libraries(${PROJECT_NAME}_calibrator
  ${PROJECT_NAME}_common
  ${VISP_LIBRARIES}
  "${cpp_typesupport_target}"
)
ament_target_dependencies(${PROJECT_NAME}_calibrator rclcpp std_msgs sensor_msgs geometry_msgs visp_bridge)

add_executable(${PROJECT_NAME}_client
  src/client.cpp
  src/client_main.cpp
)

target_link_libraries(${PROJECT_NAME}_client
  ${PROJECT_NAME}_common
  ${VISP_LIBRARIES}
  "${cpp_typesupport_target}"
)

ament_target_dependencies(${PROJECT_NAME}_client rclcpp std_msgs sensor_msgs geometry_msgs visp_bridge)

install(
  TARGETS
  ${PROJECT_NAME}_calibrator
  ${PROJECT_NAME}_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
