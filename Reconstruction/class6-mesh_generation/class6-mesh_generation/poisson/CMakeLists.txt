cmake_minimum_required(VERSION 2.8)
project(poisson)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=c++11")

include_directories(${PROJECT_SOURCE_DIR}/src)

set(HEADERS
        src/Allocator.h
        src/Array.h
        src/BinaryNode.h
        src/BSplineData.h
        src/CmdLineParser.h
        src/Factor.h
        src/FunctionData.h
        src/Geometry.h
        src/Hash.h
        src/MarchingCubes.h
        src/MAT.h
        src/MemoryUsage.h
        src/MultiGridOctreeData.h
        src/MyTime.h
        src/Octree.h
        src/Ply.h
        src/PointStream.h
        src/Polynomial.h
        src/PPolynomial.h
        src/SparseMatrix.h
        src/Vector.h
        src/surfaceTrimmer.h
        src/PoissonRecon.h
        )

set(SOURCE_FILES
        src/Array.inl
        src/BSplineData.inl
        src/CmdLineParser.cpp
        src/Factor.cpp
        src/FunctionData.inl
        src/Geometry.cpp
        src/Geometry.inl
        src/MarchingCubes.cpp
        src/MAT.inl
        src/MultiGridOctreeData.inl
        src/MultiGridOctreeData.IsoSurface.inl
        src/MultiGridOctreeData.SortedTreeNodes.inl
        src/Octree.inl
        src/PlyFile.cpp
        src/PointStream.inl
        src/Polynomial.inl
        src/PPolynomial.inl
        src/SparseMatrix.inl
        src/Vector.inl
        src/SurfaceTrimmer.cpp
        src/PoissonRecon.cpp
        )

add_library(poisson  ${HEADERS} ${SOURCE_FILES} ${SOURCE_FILES})
set_target_properties(poisson PROPERTIES VERSION 1.0 SOVERSION 1)
add_library(poisson_a STATIC
        ${SOURCE_FILES}
        ${HEADERS})
set_target_properties(poisson_a PROPERTIES OUTPUT_NAME "poisson" )
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib )

INSTALL(TARGETS poisson
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
INSTALL (FILES ${HEADERS} DESTINATION include/poisson)
INSTALL (FILES ${RESOURCE_SOURCES} DESTINATION share/poisson)
