cmake_minimum_required(VERSION 3.5)
project(visp_megapose)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visp_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(VISP REQUIRED COMPONENTS visp_core visp_detection dnn_tracker gui io)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
# service message
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Init.srv"
  "srv/Track.srv"
  "srv/Render.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)
# service message

add_executable(megapose_client src/megapose_client.cpp)
ament_target_dependencies(megapose_client
 geometry_msgs
 std_msgs
 rclcpp
 sensor_msgs
 visp_bridge
 tf2_ros
 tf2
 VISP
 image_transport
 cv_bridge
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(megapose_client "${cpp_typesupport_target}")
install(
  TARGETS
  megapose_client
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/megapose_server_.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)
install(
    DIRECTORY data
    DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
