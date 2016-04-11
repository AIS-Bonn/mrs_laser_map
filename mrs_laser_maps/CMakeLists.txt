cmake_minimum_required(VERSION 2.8.3)
project(mrs_laser_maps)
   
set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})

find_package(catkin REQUIRED COMPONENTS
	roscpp
	pcl_conversions
	pcl_ros
	pcl_msgs
	cmake_modules
	message_generation
	eigen_conversions
)

find_package(PCL REQUIRED)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})
                                                                                                    
# boost required
find_package(Boost REQUIRED COMPONENTS program_options thread system signals regex filesystem) 
include_directories(${Boost_INCLUDE_DIRS}) 

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

SET (SRCS
	src/map_multiresolution.cpp
	src/multiresolution_surfel_registration.cpp
)

add_library(${PROJECT_NAME} ${SRCS} )
target_link_libraries(${PROJECT_NAME} ${TBB_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
