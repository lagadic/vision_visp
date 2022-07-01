# add library
add_library(${PROJECT_NAME} 3dpose.cpp camera.cpp image.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${VISP_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${VISP_LIBRARIES})

target_link_libraries(${PROJECT_NAME} PRIVATE
  rcpputils::rcpputils)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
