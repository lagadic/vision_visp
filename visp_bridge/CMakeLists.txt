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

add_subdirectory(src)
add_subdirectory(tool)

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
