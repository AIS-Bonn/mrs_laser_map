
cmake_minimum_required(VERSION 2.8)
project(cloud_compression)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	pcl_ros
	pcl_conversions
	nodelet
	message_generation
	std_msgs
)

find_package(PCL REQUIRED)

add_message_files(FILES
	CompressedCloud.msg
)

generate_messages(DEPENDENCIES
	std_msgs
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES cloud_compression
)

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} include)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

add_library(cloud_compression
	src/compression.cpp
)
target_link_libraries(cloud_compression
	${catkin_LIBRARIES}
)

add_library(xyz_compression
	src/xyz_compression.cpp
)
target_link_libraries(xyz_compression
	${catkin_LIBRARIES}
	${PCL_LIBRARY_DIRS}
	cloud_compression
)
add_dependencies(xyz_compression
	${PROJECT_NAME}_generate_messages_cpp
)

add_library(xyz_decompression
	src/xyz_decompression.cpp
)
target_link_libraries(xyz_decompression
	${catkin_LIBRARIES}
	${PCL_LIBRARY_DIRS}
	cloud_compression
)
add_dependencies(xyz_decompression
	${PROJECT_NAME}_generate_messages_cpp
)
