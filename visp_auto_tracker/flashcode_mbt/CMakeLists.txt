cmake_minimum_required(VERSION 3.0.0)

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
add_subdirectory(detectors)
include_directories(detectors)
include_directories(auto_tracker)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_library(auto_tracker 
			libauto_tracker/states.hpp 
			libauto_tracker/events.h 
			libauto_tracker/tracking.h 
			libauto_tracker/tracking.cpp 
			libauto_tracker/logfilewriter.hpp 
			libauto_tracker/threading.h 
			libauto_tracker/threading.cpp)
ADD_EXECUTABLE( tracking examples/complex.cpp )
TARGET_LINK_LIBRARIES( tracking auto_tracker ${OpenCV_LIBS} qrcode_detector datamatrix_detector dmtx zbar boost_program_options cmd_line boost_thread)

ADD_EXECUTABLE( tracking_simple examples/simple.cpp )
TARGET_LINK_LIBRARIES( tracking_simple auto_tracker ${OpenCV_LIBS} qrcode_detector datamatrix_detector dmtx zbar boost_program_options cmd_line boost_thread)
