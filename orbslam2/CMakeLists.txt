cmake_minimum_required(VERSION 2.8)
project(orbslam2_core)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

add_definitions(-w)

option(WITH_ORB_C "WITH_ORB_C" OFF)
option(WITH_ORB_BFM "WITH_ORB_BFM" OFF)

set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -Wall  -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

find_package(Eigen3 3.1.0 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#set( OpenCV_DIR "/opt/opencv_320_r/share/OpenCV" )
find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()
include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND LIBS ${OpenCV_LIBS})

# find_package(G2O REQUIRED)
# if(G2O_FOUND)
#    include_directories(${G2O_INCLUDE_DIR})
#    link_directories($ENV{G2O_ROOT}/lib)
#    set(G2O_LIBRARIES g2o_core g2o_stuff g2o_types_sba g2o_types_sim3 g2o_solver_csparse g2o_csparse_extension)
# endif()
# list(APPEND LIBS ${G2O_LIBRARIES})
include_directories(${PROJECT_SOURCE_DIR}/../Thirdparty/g2o)
list(APPEND LIBS ${PROJECT_SOURCE_DIR}/../Thirdparty/g2o/lib/libg2o.so)

find_package(DBoW2 REQUIRED)
list(APPEND LIBS ${DBoW2_LIBRARIES})

find_package(Pangolin REQUIRED)
if(Pangolin_FOUND)
   include_directories(${Pangolin_INCLUDE_DIRS})
endif()
list(APPEND LIBS ${Pangolin_LIBRARIES})

include_directories(${PROJECT_SOURCE_DIR}/include)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

list(APPEND SRC_LIST
   src/System.cc
   src/Tracking.cc
   src/LocalMapping.cc
   src/LoopClosing.cc
   src/ORBextractor.cc
   src/ORBmatcher.cc
   src/FrameDrawer.cc
   src/Converter.cc
   src/MapPoint.cc
   src/KeyFrame.cc
   src/Map.cc
   src/MapDrawer.cc
   src/Optimizer.cc
   src/PnPsolver.cc
   src/Frame.cc
   src/KeyFrameDatabase.cc
   src/Sim3Solver.cc
   src/Initializer.cc
   src/Viewer.cc
   src/OrbLine.cc
)
if(WITH_ORB_C)
   add_definitions(-DWITH_ORB_C)
   list(APPEND SRC_LIST src/orb_extractor_c.cc)
endif()

if(WITH_ORB_BFM)
   add_definitions(-DWITH_ORB_BFM)
endif()

add_library(${PROJECT_NAME} SHARED ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${LIBS})


