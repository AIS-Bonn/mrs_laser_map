cmake_minimum_required(VERSION 2.8.3)
project(ais_laser_filters)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

##############################################################################
# Find dependencies
##############################################################################

set(THIS_PACKAGE_ROS_DEPS cmake_modules sensor_msgs roscpp pluginlib angles tf eigen_conversions tf_conversions pcl_ros)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS})
find_package(Boost REQUIRED COMPONENTS system signals)

find_package(PCL REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
  
##############################################################################
# Define package
##############################################################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
  DEPENDS scan_to_scan_filter_chain
  )

##############################################################################
# Build
##############################################################################

add_library(mod_laser_scan_filters src/laser_scan_filters.cpp)
target_link_libraries(mod_laser_scan_filters ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(ais_pointcloud_filters src/pointcloud_filters.cpp)
target_link_libraries(ais_pointcloud_filters ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable (cloud_to_cloud_filter_chain src/cloud_to_cloud_filter_chain.cpp)
target_link_libraries(cloud_to_cloud_filter_chain ${catkin_LIBRARIES} ${Boost_LIBRARIES})

##############################################################################
# Install
##############################################################################

install(TARGETS mod_laser_scan_filters 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES laser_filters_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
