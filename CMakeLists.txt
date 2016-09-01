CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
project(measure_base)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_VERBOSE_MAKEFILE OFF)
set(CMAKE_CXX_COMPILER "g++")

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#set(PCL_DIR /home/jyi/Devel/pcl/180/share/pcl-1.8)
set(PCL_DIR /home/jyi/Devel/pcl/share/pcl-1.8)
set(OpenCV_DIR /home/jyi/Devel/OpenCV/2412/share/OpenCV)
SET(VTK_DIR /home/jyi/Devel/VTK/lib/cmake/vtk-6.3)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
FIND_PACKAGE(VTK REQUIRED)

SET(G2O_ROOT /home/jyi/Devel/g2o)
set(G2o_LIBRARIES_DIR ${G2O_ROOT}/lib)
LIST( APPEND CMAKE_MODULE_PATH ${G2O_ROOT}/cmake_modules )
FIND_PACKAGE(G2O REQUIRED)
# CSparse
FIND_PACKAGE(CSparse)
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )
LINK_DIRECTORIES(${G2o_LIBRARIES_DIR} ${CSPARSE_LIBRARIES_DIR})

set(Libconfig_INCLUDE_DIRS /home/jyi/Devel/0000Small/libconfig/include)
set(Libconfig_LIB_DIR /home/jyi/Devel/0000Small/libconfig/lib)
set(Libconfig_LIBS libconfig.a libconfig++.a)

ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS} ${Libconfig_INCLUDE_DIRS})
LINK_DIRECTORIES(${Libconfig_LIB_DIR} ${PCL_LIBRARY_DIRS})
LINK_LIBRARIES(${Libconfig_LIBS})

FIND_PACKAGE(OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

link_directories(${PROJECT_SOURCE_DIR}/lib)
include_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)
