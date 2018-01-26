cmake_minimum_required(VERSION 3.5)
project(homework_L5)


set(CMAKE_CXX_STANDARD 11)
set( CMAKE_BUILD_TYPE "Debug" )
#set( CMAKE_CXX_FLAGS "-O3" )

include_directories( "/usr/include/eigen3" )

find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

add_executable(computeORB computeORB.cpp)
target_link_libraries( computeORB ${Pangolin_LIBRARIES} ${OpenCV_LIBS})


add_executable(E2Rt E2Rt.cpp)
target_link_libraries( E2Rt ${Sophus_LIBRARIES} )

add_executable(GN-BA GN-BA.cpp)
target_link_libraries( GN-BA  ${Sophus_LIBRARIES} )














#set(${Sophus_LIBRARIES} /home/liyubo/liyubo/slambook/3rdparty/Sophus/build/libSophus.so)
#include_directories( /home/liyubo/liyubo/slambook/3rdparty/Sophus/ )