cmake_minimum_required(VERSION 2.8)
# 声明一个 cmake 工程
project( HelloSLAM )

#set( CMAKE_BUILD_TYPE "Release" )

include_directories( "include" )
#.a 静态库 STATIC .so 动态库 SHARED
add_library( libhello SHARED src/hello.cpp )

add_executable( sayhello useHello.cpp )

target_link_libraries( sayhello libhello )

INSTALL(TARGETS  libhello 
#LIBRARY 动态库 #ARCHIVE 静态库
LIBRARY DESTINATION /usr/local/lib 
)
INSTALL(DIRECTORY include/ DESTINATION /usr/local/include
)

