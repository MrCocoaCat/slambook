cmake_minimum_required(VERSION 2.8)
project(mesh_generation)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})

add_subdirectory(poisson)
add_subdirectory(mesh_generation)
add_subdirectory(testing)



