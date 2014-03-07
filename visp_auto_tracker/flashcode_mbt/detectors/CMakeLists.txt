set(DETECTORS_BASE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(DETECTORS_BASE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/detector_base.cpp)

add_subdirectory(datamatrix)
add_subdirectory(qrcode)