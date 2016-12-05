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

#ifndef _GRAPH_PUBLISHER_H_
#define _GRAPH_PUBLISHER_H_

#include <string>
#include <fstream>

// 
// #include <tf/transform_listener.h>
// #include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>

#include <cloud_compression/octree_pointcloud_compression.h>


#include <mrs_laser_mapping/slam_graph.h>

#include <mrs_laser_maps/map_multiresolution.h>
#include <mrs_laser_mapping/color_utils.h>
#include <mrs_laser_mapping/MultiResolutionMapMsg.h>

namespace mrs_laser_mapping
{

template <typename MapPointType, typename MapType>
class GraphPublisher
{
public:

  typedef boost::shared_ptr<MapType> MapPtr;
  
  typedef typename pcl::PointCloud<MapPointType> PointCloud;
  typedef typename PointCloud::Ptr PointCloudPtr;
  
  typedef mrs_laser_mapping::SlamGraph GraphType;
  typedef GraphType::Ptr GraphPtr;
  GraphPublisher();

  ~GraphPublisher();

  static GraphPublisher* getInstance();

  void publishOdometryGraph(GraphPtr graph, nav_msgs::Odometry odometry_msg );
  
  void publishSubGraph(GraphPtr graph, nav_msgs::Odometry odometry_msg );

  void publishSLAMGraph(GraphPtr graph);
  
  void publishGraph(GraphPtr graph, bool compress = false);

  void resetNodeCounter()
  {
    node_counter_ = 0;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

private:
  static GraphPublisher* instance_;

  ros::Publisher pub_slam_graph_marker_;
  ros::Publisher pub_map_;
  ros::Publisher pub_map_downsampled_;
  ros::Publisher pub_reference_keyframe_;
  ros::Publisher pub_keyframes_;
  ros::Publisher pub_keyframe_transforms_;
  ros::Publisher pub_odometry_;
  ros::Publisher pub_local_map_;
  
  std::string frame_id_slam_map_;
  std::string frame_id_odometry_;
  
  uint32_t run_id_;

  int node_counter_;
  
  std::vector<nav_msgs::Odometry> odometry_msgs_;
  
  ros::Time start_time_;
  
  typedef cloud_compression::OctreePointCloudCompression<pcl::PointXYZRGB> Compression;
};
}

#include <mrs_laser_mapping/impl/graph_publisher.hpp>

#endif
