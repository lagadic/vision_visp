cmake_minimum_required(VERSION 3.5)
project(visp_hand2eye_calibration)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visp_bridge REQUIRED)
find_package(VISP REQUIRED COMPONENTS visp_core visp_vision)
find_package(Boost REQUIRED COMPONENTS system thread)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TransformArray.msg"
  "srv/ComputeEffectorCamera.srv"
  "srv/ComputeEffectorCameraQuick.srv"
  "srv/Reset.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

add_library(${PROJECT_NAME}_common
  src/names.cpp
  src/names.h
)
ament_target_dependencies(${PROJECT_NAME}_common
  sensor_msgs geometry_msgs rclcpp
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_executable(${PROJECT_NAME}_calibrator
  src/calibrator.cpp
  src/calibrator.h
  src/calibrator_main.cpp
)
target_link_libraries(${PROJECT_NAME}_calibrator
  ${PROJECT_NAME}_common
  ${Boost_LIBRARIES}
  ${VISP_LIBRARIES}
  "${cpp_typesupport_target}"
)
ament_target_dependencies(${PROJECT_NAME}_calibrator rclcpp std_msgs sensor_msgs geometry_msgs visp_bridge)

#add_executable(${PROJECT_NAME}_client
#  src/client.cpp
#  src/client.h
#  src/client_main.cpp
#)
#target_link_libraries(${PROJECT_NAME}_client
#  ${PROJECT_NAME}_common
#  ${VISP_LIBRARIES}
#  "${cpp_typesupport_target}"
#)
#ament_target_dependencies(${PROJECT_NAME}_client rclcpp std_msgs sensor_msgs geometry_msgs visp_bridge)

#install(
#  TARGETS
#    ${PROJECT_NAME}_calibrator
#    ${PROJECT_NAME}_client
#    ${PROJECT_NAME}_common
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
