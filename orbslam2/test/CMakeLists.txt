cmake_minimum_required(VERSION 2.8)
project(orbslam2_core)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

add_definitions(-w)

option(WITH_ORB_C "" ON)
if(WITH_ORB_C)
   add_definitions(-DWITH_ORB_C)
endif()

set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -Wall  -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()
include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND LIBS ${OpenCV_LIBS})

include_directories(../include)

add_executable(test_orb_detector test_orb_detector.cpp ../src/ORBextractor.cc ../src/orb_extractor_c.cc)
target_link_libraries(test_orb_detector ${LIBS})