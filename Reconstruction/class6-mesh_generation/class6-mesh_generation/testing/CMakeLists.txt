cmake_minimum_required(VERSION 2.8)
project(testing_mesh_generation)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})


# OPenCV
# ---[ OpenCV
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND) # if not OpenCV 3.x, then imgcodecs are not found
    find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)
endif()
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV found (${OpenCV_CONFIG_PATH})")

include_directories(../poisson/src)
include_directories(../mesh_generation/include)

add_executable(testing_mesh_generation mesh_generation.cpp )
target_link_libraries(testing_mesh_generation poisson meshGeneration ${OpenCV_LIBRARIES})



