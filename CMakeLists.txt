
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

if(NOT LIBYUV_INCLUDE_DIR)
	message(WARNING "libyuv not found, not building xtion_grabber")
	return()
endif()

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

include_directories(${LIBYUV_INCLUDE_DIR})

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
	${LIBYUV_LIBRARY}
)
