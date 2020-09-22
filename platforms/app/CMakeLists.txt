cmake_minimum_required(VERSION 2.8)
project(orbslam2_app)

set(CMAKE_BUILD_TYPE "Release")
add_definitions(-w)

# orbslam2 core
set(PROJECT_ROOT "../../")
add_subdirectory(${PROJECT_ROOT}/orbslam2 orbslam2.core)
include_directories(${PROJECT_ROOT}/orbslam2/include)

# set(CMAKE_CXX_STANDARD 11)
# add_compile_options(-std=c++11)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions(-DCOMPILEDWITHC11)

find_package(OpenCV)
if(OpenCV_FOUND)
	include_directories(${OpenCV_INCLUDE_DIRS})
	link_libraries(${OpenCV_LIBS})
endif()

find_package(Eigen3 3.1.0 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(rgbd_tum RGB-D/rgbd_tum.cc)
target_link_libraries(rgbd_tum orbslam2_core)

add_executable(stereo_kitti Stereo/stereo_kitti.cc)
target_link_libraries(stereo_kitti orbslam2_core)

add_executable(stereo_euroc Stereo/stereo_euroc.cc)
target_link_libraries(stereo_euroc orbslam2_core)

add_executable(mono_tum Monocular/mono_tum.cc)
target_link_libraries(mono_tum orbslam2_core)

add_executable(mono_kitti Monocular/mono_kitti.cc)
target_link_libraries(mono_kitti orbslam2_core)

add_executable(mono_euroc Monocular/mono_euroc.cc)
target_link_libraries(mono_euroc orbslam2_core)

add_executable(myslam GHC/myslam.cpp)
target_link_libraries(myslam orbslam2_core)
