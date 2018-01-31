cmake_minimum_required(VERSION 3.8)
project(lesson6)

set(CMAKE_CXX_STANDARD 11)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(G2O REQUIRED)
find_package(CSparse REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
        "/usr/include/eigen3/"
)
add_executable(optical_flow optical_flow.cpp)
target_link_libraries(optical_flow
        ${OpenCV_LIBS}
        )

add_executable(myoptical_flow myoptical_flow.cpp)
target_link_libraries(myoptical_flow
        ${OpenCV_LIBS}
        )

add_executable(direct_method direct_method.cpp)
target_link_libraries(direct_method
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES}
        )