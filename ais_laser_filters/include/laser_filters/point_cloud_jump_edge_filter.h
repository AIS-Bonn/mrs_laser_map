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

#ifndef POINT_CLOUD_JUMP_EDGE_FILTER_H
#define POINT_CLOUD_JUMP_EDGE_FILTER_H
/**
\author David Droeschel
@b PointCloudJumpEdgeFilter takes clouds and filters for jump edges.  

**/

#include "point_types.h"

#include "filters/filter_base.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include "ros/ros.h"

#include <pcl/common/time.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>

#include <angles/angles.h>


namespace ais_laser_filters
{

class PointCloudJumpEdgeFilter : public filters::FilterBase<sensor_msgs::PointCloud2>
{
  typedef velodyne_pointcloud::PointXYZIR PointType;
  
public:
  PointCloudJumpEdgeFilter() 
  {
    
  }

  bool configure()
  {
    
    if(!getParam("sensor_frame", sensor_frame_id_)) 
    {
      ROS_ERROR("PointCloudJumpEdgeFilter: sensor_frame must be set."); 
      return false;
    }      
    if (!getParam(std::string("min_angle"), min_angle_))
    {
      ROS_ERROR("Error: PointCloudJumpEdgeFilter was not given min_angle.\n");
      return false;
    }
    if (!getParam(std::string("max_angle"), max_angle_))
    {
      ROS_ERROR("Error: PointCloudJumpEdgeFilter was not given min_angle.\n");
      return false;
    }
    if (!getParam(std::string("angle_increment"), angle_increment_))
    {
      ROS_ERROR("Error: PointCloudJumpEdgeFilter was not given angle_increment.\n");
      return false;
    }
    if (!getParam(std::string("window"), window_))
    {
      ROS_ERROR("Error: PointCloudJumpEdgeFilter was not given window.\n");
      return false;
    }
    neighbors_ = 0;  // default value
    if (!getParam(std::string("neighbors"), neighbors_))
    {
      ROS_INFO("Error: PointCloudJumpEdgeFilter was not given neighbors.\n");
    }
    max_filter_radius_ = std::numeric_limits< double >::max();
    if (!getParam(std::string("max_filter_radius"), max_filter_radius_))
    {
      ROS_INFO("Error: PointCloudJumpEdgeFilter was not max filter radius.\n");
    }
    
    return true;
  }

  virtual ~PointCloudJumpEdgeFilter()
  { 

  }

  bool update(const sensor_msgs::PointCloud2& input_scan, sensor_msgs::PointCloud2& filtered_scan)
  {
    pcl::StopWatch watch;
    if(&input_scan == &filtered_scan){
      ROS_ERROR("This filter does not currently support in place copying");
      return false;
    }
    sensor_msgs::PointCloud2 laser_cloud;
    
    tf::StampedTransform sensor_frame_transform;
    try
    {
      tf_.lookupTransform(sensor_frame_id_, input_scan.header.frame_id, input_scan.header.stamp, sensor_frame_transform);
    }
    catch(tf::TransformException& ex)
    {
      ROS_ERROR("Transform unavailable %s", ex.what());
      return false;
    }
    Eigen::Affine3d sensor_frame_transform_eigen;
    tf::transformTFToEigen(sensor_frame_transform, sensor_frame_transform_eigen);
    
    pcl::PointCloud<PointType>::Ptr pcl_cloud(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg<PointType>(input_scan, *pcl_cloud);
   
    std::vector<std::vector<std::pair<float,unsigned int>>> distances_by_ring;
    for (unsigned int i = 0; i < 16; ++i)
    {
      std::vector<std::pair<float,unsigned int>> v;
      distances_by_ring.push_back(v);
    }
    
    pcl::PointCloud<PointType>::Ptr pcl_cloud_range_transformed(new pcl::PointCloud<PointType>); 
    pcl::transformPointCloud(*pcl_cloud, *pcl_cloud_range_transformed, sensor_frame_transform_eigen);
    
    for ( size_t i = 0 ; i < pcl_cloud_range_transformed->points.size(); ++i )
    {
      auto& point = pcl_cloud_range_transformed->points[i];
      float dist = point.getVector3fMap().norm();
      if ( dist < max_filter_radius_*2.0)
	distances_by_ring[point.ring].push_back(std::make_pair(dist, i));
    }
    
    pcl::IndicesPtr indices_all_rings (new std::vector <int>);
    for (unsigned int i = 0; i < 16; ++i)
    {
      pcl::IndicesPtr indices (new std::vector <int>);
      filter_jump_edges(distances_by_ring[i], indices);
      indices_all_rings->insert(indices_all_rings->end(), indices->begin(), indices->end());
    }
    std::sort(indices_all_rings->begin(), indices_all_rings->end());
    indices_all_rings->erase(std::unique(indices_all_rings->begin(), indices_all_rings->end()), indices_all_rings->end());
    
    auto inidices_iterator = indices_all_rings->begin();
    
    pcl::PointCloud<PointType>::Ptr pcl_output_cloud(new pcl::PointCloud<PointType>); 
    for ( int i = 0; i < pcl_cloud->points.size(); ++i)
    {
      if ( inidices_iterator != indices_all_rings->end() && *inidices_iterator == i )
      {
	inidices_iterator++;
      }
      else 
      {
	pcl_output_cloud->points.push_back(pcl_cloud->points[i]);
      }
    }
    pcl::toROSMsg<PointType>(*pcl_output_cloud, filtered_scan);
    filtered_scan.header.frame_id = input_scan.header.frame_id;
    filtered_scan.header.stamp = input_scan.header.stamp;
  
    ROS_DEBUG_STREAM("PointCloudJumpEdgeFilter took:" << watch.getTime());
    return true;
  }

  
  /** @brief calculate the perpendicular angle at the end of r1 to get to r2
   * See http://en.wikipedia.org/wiki/Law_of_cosines */
  inline double getAngleWithViewpoint(float r1, float r2, float included_angle)
  {
    return atan2(r2 * sin(included_angle), r1 - r2 * cos(included_angle));
  }

  ////////////////////////////////////////////////////////////////////////////////
  /** \brief Filter shadow points based on 3 global parameters: min_angle, max_angle
   * and window. {min,max}_angle specify the allowed angle interval (in degrees)
   * between the created lines (see getAngleWithViewPoint). Window specifies how many
   * consecutive measurements to take into account for one point.
   * \param cloud the input point cloud
   * \param indices indices with detected points 
   */
  bool filter_jump_edges(const std::vector<std::pair<float, unsigned int>>& distances, pcl::IndicesPtr indices)
  {
    // For each point in the current line scan
    for (unsigned int i = 0; i < distances.size(); i++)
    {
      if (distances[i].first < max_filter_radius_)
      {
        for (int y = -window_; y < window_ + 1; y++)
        {
          int j = i + y;
          if (j < 0 || j >= (int)distances.size() || (int)i == j)
          {  // Out of scan bounds or itself
            continue;
          }

          double angle = abs(angles::to_degrees(
              getAngleWithViewpoint(distances[i].first, distances[j].first, y * angle_increment_)));
// 	  ROS_INFO_STREAM_THROTTLE(0.1, "angles: " << angle << " " << distances[i].first << " " << distances[j].first);
          if (angle < min_angle_ || angle > max_angle_)
          {
            for (int index = std::max<int>(i - neighbors_, 0);
                 index <= std::min<int>(i + neighbors_, (int)distances.size() - 1); index++)
	    {
              //	if (cloud->points[i] < cloud->points[index]) // delete neighbor if they are farther away (note not
              //self)
              indices->push_back(distances[index].second);
	    }
          }
        }
      }
    }
    ROS_DEBUG("PointCloudJumpEdgeFilter detected %d Points from scan with min angle: %.2f, max angle: %.2f, neighbors: %d, "
              "and window: %d",
              (int)indices->size(), min_angle_, max_angle_, neighbors_, window_);
   
    return true;
  }


private:
  tf::TransformListener tf_;

  std::string sensor_frame_id_;  
  double min_angle_, max_angle_;  // Filter angle threshold
  double angle_increment_;
  int window_, neighbors_;
  double max_filter_radius_;
  
} ;

}

#endif // POINT_CLOUD_JUMP_EDGE_FILTER_H
