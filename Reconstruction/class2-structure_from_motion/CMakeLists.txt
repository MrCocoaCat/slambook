
#设置最低的cmake版本
cmake_minimum_required(VERSION 2.8)
#定义工程名称
project(structure_from_motion)
#使用C++11
set(CMAKE_CXX_STANDARD 11)

IF(MSVC)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP") ### Enable parallel compilation for Visual Studio
    SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR} )
    SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR} )
ELSE(MSVC)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive -std=c++11") #### Libigl requires a modern C++ compiler that supports c++11
    #SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY "../" )
ENDIF(MSVC)

include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
)
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# Eigen (required)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions( -DEIGEN_USE_NEW_STDVECTOR -DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)

#设置OpenCV库
# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)
# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

if(CMAKE_VERSION VERSION_LESS "2.8.11")
    # Add OpenCV headers location to your include paths
    include_directories(${OpenCV_INCLUDE_DIRS})
endif()


# ceres library
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIR})


set(FUNDEMENTAL_SRC_FILES fundemental.cxx robust_matcher.cxx robust_matcher.h functions.h functions.cxx)
add_executable(fundemental ${FUNDEMENTAL_SRC_FILES})
target_link_libraries(fundemental ${OpenCV_LIBS})


set(HOMOGRAPHY_SRC_FILES robust_matcher.cxx robust_matcher.h homography.cxx functions.h functions.cxx)
add_executable(homography ${HOMOGRAPHY_SRC_FILES})
target_link_libraries(homography ${OpenCV_LIBS})

set(ESSIENTIAL_SRC_FILES essential.cxx robust_matcher.cxx robust_matcher.h functions.h functions.cxx )
add_executable(essential ${ESSIENTIAL_SRC_FILES})
target_link_libraries(essential ${OpenCV_LIBS})

set(TRIANGULATION_SRC_FILES triangulation.cxx robust_matcher.cxx robust_matcher.h functions.h functions.cxx )
add_executable(triangulation ${TRIANGULATION_SRC_FILES})
target_link_libraries(triangulation ${OpenCV_LIBS})


set(PNP_SRC_FILES PnP.cxx)
add_executable(pnp ${PNP_SRC_FILES})
target_link_libraries(pnp ${OpenCV_LIBS})


set(BA_SRC_FILES bundle_adjustment.cxx
        robust_matcher.h
        robust_matcher.cxx
        functions.cxx
        functions.h
        snavely_reprojection_error.h
        bal_problem.h
        bal_problem.cc
        random.h)
add_executable(bundle_adjustment ${BA_SRC_FILES})
target_link_libraries(bundle_adjustment ${OpenCV_LIBS} ${CERES_LIBRARIES})


