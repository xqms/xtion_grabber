
cmake_minimum_required(VERSION 2.8)

project(xtion_grabber)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	image_transport
	nodelet
	cv_bridge
	camera_info_manager
	cmake_modules
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

set(CMAKE_CXX_FLAGS "-Wall -g -march=native -std=c++11")

find_path(LIBYUV_INCLUDE_DIR libyuv.h)
find_library(LIBYUV_LIBRARY yuv)

if(LIBYUV_INCLUDE_DIR)
	include_directories(${LIBYUV_INCLUDE_DIR})
	add_definitions(-DHAVE_LIBYUV=1)

	set(LIBYUV_LIBRARIES ${LIBYUV_LIBRARY})
endif()

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

enable_language(ASM)
set_property(SOURCE src/generate_pointcloud.s PROPERTY LANGUAGE ASM)

add_library(xtion_grabber
	src/xtion_grabber.cpp
	src/generate_pointcloud.cpp
	src/generate_pointcloud.s
	src/depth_filler.cpp
)
target_link_libraries(xtion_grabber
	${catkin_LIBRARIES}
	${LIBYUV_LIBRARIES}
)
