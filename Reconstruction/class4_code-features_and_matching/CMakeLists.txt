
#设置最低的cmake版本
cmake_minimum_required(VERSION 2.8)
#定义工程名称
project(features_and_matching)
#使用C++11
set(CMAKE_CXX_STANDARD 11)

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


set(FUNCTIONS functions.h functions.cxx)

#设置源文件
set(FEATURES_SRC_FILES features.cxx ${FUNCTIONS})
add_executable(feature_detection ${FEATURES_SRC_FILES})
# 链接OpenCV库文件倒应用程序
target_link_libraries(feature_detection ${OpenCV_LIBS})


set(BFM_MATCHING_SRC_FILES bfm_matcher.cxx ${FUNCTIONS})
add_executable(bfm_matcher ${BFM_MATCHING_SRC_FILES})
target_link_libraries(bfm_matcher ${OpenCV_LIBS})

set(KNN_MATCHING_SRC_FILES knn_matcher.cxx ${FUNCTIONS})
add_executable(knn_matcher ${KNN_MATCHING_SRC_FILES})
target_link_libraries(knn_matcher ${OpenCV_LIBS})


