cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})


add_definitions(-DTRILIBRARY )

# find Qt package
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED QUIET)



#include poisson
include_directories(../poisson/src)


include_directories(include)

set(HEADERS include/data_type.h
            include/igit_dataIO.h
            include/igit_mesh_generation.h
            include/triangle.h
            include/vec.h)

set(SOURCES src/data_type.cpp
            src/igit_dataIO.cpp
            src/triangle.cpp
            src/igit_mesh_generation.cpp
            src/vec.cpp)


add_library(meshGeneration SHARED ${HEADERS} ${SOURCES} ../testing/mesh_generation.cpp)
target_link_libraries(meshGeneration poisson)
set_target_properties(meshGeneration PROPERTIES VERSION 1.0 SOVERSION 1)
qt5_use_modules(meshGeneration Core Gui Widgets)

add_library(meshGeneration_a STATIC ${HEADERS} ${SOURCES})
set_target_properties(meshGeneration_a PROPERTIES OUTPUT_NAME "meshGeneration" )
target_link_libraries(meshGeneration_a poisson)
qt5_use_modules(meshGeneration_a Core Gui Widgets)

