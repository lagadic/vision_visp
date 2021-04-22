cmake_minimum_required(VERSION 3.0.0)
project(visp_hand2eye_calibration)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  image_proc
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  visp_bridge
)

# ViSP cannot be found by Catkin.
# see https://github.com/ros/catkin/issues/606
find_package(VISP REQUIRED)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Generate messages and services.
add_message_files(
  DIRECTORY
    msg
  FILES
    TransformArray.msg
  )
add_service_files(
  DIRECTORY
    srv
  FILES
    compute_effector_camera.srv
    compute_effector_camera_quick.srv
    reset.srv
  )
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    roscpp
    sensor_msgs
    std_msgs
)

add_library(${PROJECT_NAME}_common
  src/names.cpp
  src/names.h
)
target_link_libraries(${PROJECT_NAME}_common
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  )

add_executable(${PROJECT_NAME}_calibrator
  src/calibrator.cpp
  src/calibrator.h
  src/calibrator_main.cpp
)
target_link_libraries(${PROJECT_NAME}_calibrator
  ${PROJECT_NAME}_common
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  )
add_dependencies(${PROJECT_NAME}_calibrator
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencpp)

add_executable(${PROJECT_NAME}_client
  src/client.cpp
  src/client.h
  src/client_main.cpp
)
target_link_libraries(${PROJECT_NAME}_client
  ${PROJECT_NAME}_common
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  )
add_dependencies(${PROJECT_NAME}_client
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencpp)

install(
  TARGETS
    ${PROJECT_NAME}_calibrator
    ${PROJECT_NAME}_client
    ${PROJECT_NAME}_common
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

