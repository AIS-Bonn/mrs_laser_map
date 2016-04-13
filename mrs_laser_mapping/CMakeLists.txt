cmake_minimum_required(VERSION 2.8.3)
project(mrs_laser_mapping)
   
set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})


find_package(catkin REQUIRED COMPONENTS
	roscpp
	sensor_msgs
	std_msgs
	pcl_conversions
	pcl_ros
	pcl_msgs
	laser_geometry
	tf
	geometry_msgs
	nodelet
	nav_msgs
	visualization_msgs
	tf_conversions
	cmake_modules
	message_generation
	config_server
	interactive_markers
	cloud_compression
	eigen_conversions
	mrs_laser_maps
)

find_package(PCL REQUIRED)

add_message_files(FILES
	MultiResolutionMap.msg
	KeyFrame.msg
	KeyFrameTransforms.msg
)

add_service_files(
  FILES
  AddPointsToMap.srv
  #InterestPointsSwitchHand.srv
  #DeleteMarkedPoints.srv
  SelfFilter.srv
  AddKeyFramesByDistance.srv
) 

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
   INCLUDE_DIRS include
)
                                                                                                                                                                                                    #                                                                                                 
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})
                                                                                                    
# boost required
find_package(Boost REQUIRED COMPONENTS program_options thread system signals regex filesystem) 
include_directories(${Boost_INCLUDE_DIRS}) 

# G2O required
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR})

# TBB required
find_package(TBB REQUIRED)
include_directories(${TBB_INCLUDE_DIRS})

# For building the CHOLMOD / CSPARSE solvers
find_package(Cholmod)
include_directories(${CHOLMOD_INCLUDE_DIR})
find_package(BLAS)
find_package(LAPACK)
find_package(CSparse)
include_directories(${CSPARSE_INCLUDE_DIR})

find_package(mrs_laser_maps REQUIRED) 

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
        
SET ( SRCS
	src/trajectory_publisher.cpp 
	src/map_publisher.cpp 
)               

SET ( SLAM_SRCS
	src/mrsmap/slam.cpp 
)         

SET (SLAM_LIBS
        ${G2O_CORE_LIBRARY} 
        ${G2O_STUFF_LIBRARY} 
        ${G2O_SOLVER_CSPARSE} 
        ${G2O_SOLVER_CSPARSE_EXTENSION} 
        ${G2O_TYPES_SLAM3D} 
)
 
add_library(scan_assembler_nodelet src/nodes/scan_assembler_nodelet.cpp)
target_link_libraries(scan_assembler_nodelet ${catkin_LIBRARIES})
add_dependencies(scan_assembler_nodelet
	${PROJECT_NAME}_generate_messages_cpp
)

add_library(map_nodelet src/nodes/map_nodelet.cpp ${SRCS} )
target_link_libraries(map_nodelet ${mrs_laser_maps_LIBRARIES} ${TBB_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(map_nodelet
	${PROJECT_NAME}_generate_messages_cpp
)
 
add_library(slam_nodelet src/nodes/slam_nodelet.cpp ${SLAM_SRCS})
target_link_libraries(slam_nodelet ${SLAM_LIBS} ${mrs_laser_maps_LIBRARIES} ${TBB_LIBRARIES}  ${catkin_LIBRARIES} ) 
add_dependencies(slam_nodelet
	${PROJECT_NAME}_generate_messages_cpp
)

add_library(slam_visualizer_nodelet src/nodes/slam_visualizer_nodelet.cpp )
target_link_libraries(slam_visualizer_nodelet ${catkin_LIBRARIES})
add_dependencies(slam_visualizer_nodelet
	${PROJECT_NAME}_generate_messages_cpp
)
