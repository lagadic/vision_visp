cmake_minimum_required(VERSION 2.8.3)
project(visp_camera_calibration)

find_package(catkin REQUIRED COMPONENTS
  visp_bridge
  message_generation
  camera_calibration_parsers
  sensor_msgs
  std_msgs
  geometry_msgs
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
    CalibPointArray.msg
    CalibPoint.msg
    ImageAndPoints.msg  
    ImagePoint.msg
  )
add_service_files(
  DIRECTORY 
    srv
  FILES
    calibrate.srv
  )
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    sensor_msgs
    std_msgs
    geometry_msgs
)

add_library(visp_camera_calibration_common src/names.cpp src/names.h)

add_executable(visp_camera_calibration_calibrator
  src/calibrator.cpp
  src/calibrator.h
  src/calibrator_main.cpp)
target_link_libraries(visp_camera_calibration_calibrator visp_camera_calibration_common ${catkin_LIBRARIES})
add_dependencies(visp_camera_calibration_calibrator ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

add_executable(visp_camera_calibration_image_processing
  src/image_processing.cpp
  src/image_processing.h
  src/image_processing_main.cpp)
target_link_libraries(visp_camera_calibration_image_processing visp_camera_calibration_common ${catkin_LIBRARIES})
add_dependencies(visp_camera_calibration_image_processing ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

add_executable(visp_camera_calibration_camera
  src/camera.cpp
  src/camera.h
  src/camera_main.cpp)
target_link_libraries(visp_camera_calibration_camera visp_camera_calibration_common ${catkin_LIBRARIES})
add_dependencies(visp_camera_calibration_camera ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)

install(
  TARGETS
    visp_camera_calibration_common
    visp_camera_calibration_calibrator
    visp_camera_calibration_image_processing
    visp_camera_calibration_camera
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
