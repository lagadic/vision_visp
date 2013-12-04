cmake_minimum_required(VERSION 2.8.3)
project(visp_hand2eye_calibration)

find_package(catkin REQUIRED COMPONENTS
  visp_bridge
  message_generation
  image_proc
)

# TO FIX, VISP_DEFINITIONS should come from visp_bridge dependency
find_package(VISP REQUIRED)
add_definitions(${VISP_DEFINITIONS})

include_directories(
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

catkin_package()

add_library(visp_hand2eye_calibration_common 
  src/names.cpp
  src/names.h
)

add_executable(visp_hand2eye_calibration_calibrator 
  src/calibrator.cpp 
  src/calibrator.h 
  src/calibrator_main.cpp
)
target_link_libraries(visp_hand2eye_calibration_calibrator visp_hand2eye_calibration_common ${catkin_LIBRARIES})
add_dependencies(visp_hand2eye_calibration_calibrator ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

add_executable(visp_hand2eye_calibration_client 
  src/client.cpp 
  src/client.h 
  src/client_main.cpp
)
target_link_libraries(visp_hand2eye_calibration_client visp_hand2eye_calibration_common ${catkin_LIBRARIES})
add_dependencies(visp_hand2eye_calibration_client ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

install(
  TARGETS
    visp_hand2eye_calibration_calibrator
    visp_hand2eye_calibration_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


