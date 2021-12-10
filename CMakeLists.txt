cmake_minimum_required( VERSION 2.8 )
project( Camera_calibration )

# add c++ 11
set( CMAKE_CXX_FLAGS "-std=c++11" )

# find OpenCV
find_package( OpenCV 3 REQUIRED )
# add header file
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( Camera_Cal camera_calibration.cpp )
# link OpenCV library
target_link_libraries( Camera_Cal ${OpenCV_LIBS} )

#add_executable( Read_Yaml readyaml.cpp )
# link OpenCV library
#target_link_libraries( Read_Yaml ${OpenCV_LIBS} )




