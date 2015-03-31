cmake_minimum_required(VERSION 2.8.3)
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


add_definitions(${VISP_DEFINITIONS})

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
#link_directories(
#  ${Boost_LIBRARY_DIRS}
#  ${VISP_LIBRARY_DIRS}
#  ${catkin_LIBRARY_DIRS}
#  )

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
    message_runtime
    roscpp
    sensor_msgs
    std_msgs
    geometry_msgs
)

add_library(visp_hand2eye_calibration_common
  src/names.cpp
  src/names.h
)
target_link_libraries(visp_hand2eye_calibration_common
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  )

add_executable(visp_hand2eye_calibration_calibrator
  src/calibrator.cpp
  src/calibrator.h
  src/calibrator_main.cpp
)
target_link_libraries(visp_hand2eye_calibration_calibrator
  visp_hand2eye_calibration_common
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  )
add_dependencies(visp_hand2eye_calibration_calibrator
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencpp)

add_executable(visp_hand2eye_calibration_client
  src/client.cpp
  src/client.h
  src/client_main.cpp
)
target_link_libraries(visp_hand2eye_calibration_client
  visp_hand2eye_calibration_common
  ${catkin_LIBRARIES}
  ${roscpp_LIBRARIES}
  )
add_dependencies(visp_hand2eye_calibration_client
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencpp)

install(
  TARGETS
    visp_hand2eye_calibration_calibrator
    visp_hand2eye_calibration_client
    visp_hand2eye_calibration_common
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


