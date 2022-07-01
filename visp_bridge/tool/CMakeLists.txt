add_executable(${PROJECT_NAME}_convert_camera_parameters convert_cam_param_file.cpp)

target_include_directories(${PROJECT_NAME}_convert_camera_parameters PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${VISP_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}_convert_camera_parameters PUBLIC
  ${camera_calibration_parsers_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${VISP_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}_convert_camera_parameters PRIVATE
  rcpputils::rcpputils)

install(TARGETS ${PROJECT_NAME}_convert_camera_parameters
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
