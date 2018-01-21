cmake_minimum_required(VERSION 2.8)
project( homework4 )

set(CMAKE_CXX_STANDARD 11)
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-O3" )
include_directories( "/usr/include/eigen3" )

find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )



add_executable(disparity disparity.cpp)
target_link_libraries( disparity ${Pangolin_LIBRARIES} ${OpenCV_LIBS})

add_executable(gaussnewton gaussnewton.cpp)
target_link_libraries( gaussnewton  ${OpenCV_LIBS})

add_executable(undistort_imag undistort_image.cpp)
target_link_libraries( undistort_imag  ${OpenCV_LIBS})