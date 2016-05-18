cmake_minimum_required(VERSION 2.8.3)
project(mrs_laser_maps)
   
set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/Modules/" ${CMAKE_MODULE_PATH})

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  eigen_conversions
  message_generation
  pcl_conversions
  roscpp
  rosunit
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

# add_definitions(-DDEBUG_CELL_HITS)
                                                                                                    
# boost required
find_package(Boost REQUIRED COMPONENTS program_options thread system signals regex filesystem) 
include_directories(${Boost_INCLUDE_DIRS}) 

set (${PROJECT_NAME}_SRCS
  src/map_multiresolution.cpp
  src/multiresolution_surfel_registration.cpp
)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS} )
target_link_libraries(${PROJECT_NAME} ${TBB_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

