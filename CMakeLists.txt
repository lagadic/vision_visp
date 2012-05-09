cmake_minimum_required(VERSION 2.8)

PROJECT( tracking )
FIND_PACKAGE( OpenCV REQUIRED )
FIND_PACKAGE(VISP REQUIRED)
IF(VISP_FOUND)
  INCLUDE(${VISP_USE_FILE})
ENDIF(VISP_FOUND)

INCLUDE_DIRECTORIES(${OpenCV_DIR})

find_package( Boost REQUIRED)
link_directories ( ${Boost_LIBRARY_DIRS} )
include_directories ( ${Boost_INCLUDE_DIRS} )
add_subdirectory(cmd_line)
add_subdirectory(datamatrix)

ADD_EXECUTABLE( tracking main.cpp video.h video.cpp logitech.hpp)
TARGET_LINK_LIBRARIES( tracking ${OpenCV_LIBS} datamatrix_detector dmtx boost_program_options cmd_line )
