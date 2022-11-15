add_executable(${PROJECT_NAME}_cam_param_converter cam_param_converter.cpp)

target_include_directories(${PROJECT_NAME}_cam_param_converter PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${VISP_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}_cam_param_converter PUBLIC
  ${camera_calibration_parsers_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${VISP_LIBRARIES}
  ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}_cam_param_converter PRIVATE
  rcpputils::rcpputils)

install(TARGETS ${PROJECT_NAME}_cam_param_converter
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  ${CMAKE_SOURCE_DIR}/data
  DESTINATION share
)
