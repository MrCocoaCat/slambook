cmake_minimum_required(VERSION 2.8)
project(multiview_stitching)

if(WIN32)
    if(MSVC)
        add_definitions(-D_SCL_SECURE_NO_WARNINGS)
        add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
    endif(MSVC)
else(WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif(WIN32)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} include /usr/include/eigen3)
link_directories(${OpenCV_LINK_DIRECTORIES})

set(INCLUDES include/cvconfig.h
             include/autocalib.hpp
             include/blenders.hpp
             include/camera.hpp
             include/exposure_compensate.hpp
             include/matchers.hpp
             include/motion_estimators.hpp
             include/seam_finders.hpp
             include/timelapsers.hpp
             include/util.hpp
             include/util_inl.hpp
             include/warpers.hpp
             include/warpers_inl.hpp)

set(SOURCE_FILES
              #src/autocalib.cpp
              #src/blenders.cpp
              #src/camera.cpp
              #src/exposure_compensate.cpp
              #src/matchers.cpp
              #src/motion_estimators.cpp
              #src/precomp.hpp
              #src/seam_finders.cpp
              #src/stitcher.cpp
              #src/timelapsers.cpp
              #src/util.cpp
              #src/util_log.hpp
              #src/warpers.cpp
              src/stitching_detailed.cxx)


add_executable(multiview_stitching ${INCLUDES} ${SOURCE_FILES})

#set(OPENCV_LIBS opencv_core opencv_highgui opencv_video
#          opencv_calib3d opencv_imgcodecs opencv_stitching opencv_core  opencv_core )

target_link_libraries(multiview_stitching  ${OpenCV_LIBRARIES})


#set(TEST_BLENDER_FILES src/test_blenders.cxx src/test_precomp.hpp)
#add_executable(test_blenders ${TEST_BLENDER_FILES})
#target_link_libraries(test_blenders ${OpenCV_LIBRARIES})