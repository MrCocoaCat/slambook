cmake_minimum_required(VERSION 2.8)
# 声明一个 cmake 工程
project( HelloSLAM )

set( CMAKE_BUILD_TYPE "Release" )

include_directories("include")

add_executable( sayhello HelloSLAM.cpp )

add_library( libhello src/hello.c )

target_link_libraries(sayhello libhello)
