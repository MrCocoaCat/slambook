cmake_minimum_required(VERSION 3.5)
project(homework_L6)

set(CMAKE_CXX_STANDARD 11)

include_directories( "/usr/include/eigen3" )

find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

find_package( OpenCV )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(direct_method direct_method.cpp)
target_link_libraries( direct_method ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES})

add_executable(optical_flow optical_flow.cpp)
target_link_libraries( optical_flow ${OpenCV_LIBS} ${Sophus_LIBRARIES})