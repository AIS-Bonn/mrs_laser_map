cmake_minimum_required(VERSION 2.8.3)
project(mod_laser_filters)

##############################################################################
# Find dependencies
##############################################################################

set(THIS_PACKAGE_ROS_DEPS cmake_modules sensor_msgs roscpp pluginlib angles tf laser_geometry)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS})
find_package(Boost REQUIRED COMPONENTS system signals)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

##############################################################################
# Define package
##############################################################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES laser_scan_filters
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
  DEPENDS scan_to_scan_filter_chain
  )

##############################################################################
# Build
##############################################################################

add_library(mod_laser_scan_filters src/laser_scan_filters.cpp)
target_link_libraries(mod_laser_scan_filters ${catkin_LIBRARIES} ${Boost_LIBRARIES})


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
