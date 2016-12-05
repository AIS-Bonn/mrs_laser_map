/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Computer Science Institute VI, University of Bonn
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of University of Bonn, Computer Science Institute
 *     VI nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _MAP_PUBLISHER_H_
#define _MAP_PUBLISHER_H_

#include <pcl_ros/transforms.h>

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>

#include <mrs_laser_maps/map_multiresolution.h>
#include <mrs_laser_mapping/MultiResolutionMapMsg.h>
#include <mrs_laser_mapping/color_utils.h>


namespace mrs_laser_mapping
{
class MapPublisher
{
public:
  MapPublisher();

  ~MapPublisher();

  static MapPublisher* getInstance();

  template <typename PointType, typename MapType>
  void publishOccupiedCells(const boost::shared_ptr<MapType>& map);

  template <typename PointType, typename MapType>
  void publishCellsWithOccupancy(const boost::shared_ptr<MapType>& map);

  template <typename PointType, typename MapType>
  void publishMap(const boost::shared_ptr<MapType>& map, bool distorted = false);

  template <typename PointType, typename MapType>
  void publishMapLevelColor(const boost::shared_ptr<MapType>& map);
  
  template <typename PointType, typename MapType>
  void publishMapScanColor(const boost::shared_ptr<MapType>& map);

private:
  static MapPublisher* instance_;

  ros::NodeHandle node_handle_;
  ros::Publisher pub_marker_;
  ros::Publisher pub_map_msg_;
  ros::Publisher pub_cloud_level_color_;
  ros::Publisher pub_cloud_scan_color_;
};

// template <>
//  void MapPublisher::publishMapLevelColor<pcl::PointXYZ, mrs_laser_maps::MultiResolutionalMap<pcl::PointXYZ>>(const boost::shared_ptr<mrs_laser_maps::MultiResolutionalMap<pcl::PointXYZ>>& map)
// {
// }
// 
// template <>
//  void MapPublisher::publishMapScanColor<pcl::PointXYZ, mrs_laser_maps::MultiResolutionalMap<pcl::PointXYZ>>(const boost::shared_ptr<mrs_laser_maps::MultiResolutionalMap<pcl::PointXYZ>>& map)
// {
// }
// 

}

#include <mrs_laser_mapping/impl/map_publisher.hpp>

#endif
