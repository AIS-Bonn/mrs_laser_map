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

#ifndef POINT_CLOUD_FOOTPRINT_FILTER_H
#define POINT_CLOUD_FOOTPRINT_FILTER_H
/**
\author David Droeschel
@b PointCloudFootprintFilter takes clouds cuts out rectangular box.  

**/

#include "filters/filter_base.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include "ros/ros.h"

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>

#include "point_types.h"

namespace ais_laser_filters
{

class PointCloudFootprintFilter : public filters::FilterBase<sensor_msgs::PointCloud2>
{
  typedef velodyne_pointcloud::PointXYZIR PointType;
  
public:
  PointCloudFootprintFilter() 
  : filter_by_radius_(false),
    inscribed_radius_(0.0),
    filter_min_x_(0.0),
    filter_max_x_(0.0),
    filter_min_y_(0.0),
    filter_max_y_(0.0),
    filter_min_z_(0.0),
    filter_max_z_(0.0)
  {
    
  }

  bool configure()
  {
    if(getParam("inscribed_radius", inscribed_radius_))
    {
      ROS_INFO("PointCloudFootprintFilter uses inscribed_radius.");
      filter_by_radius_ = true;
    }
    else 
    {
      getParam("filter_min_x", filter_min_x_);
      getParam("filter_max_x", filter_max_x_);
      getParam("filter_min_y", filter_min_y_);
      getParam("filter_max_y", filter_max_y_);
      getParam("filter_min_z", filter_min_z_);
      getParam("filter_max_z", filter_max_z_);
      ROS_INFO("PointCloudFootprintFilter uses min/max x y z.");

    }
    return true;
  }

  virtual ~PointCloudFootprintFilter()
  { 

  }

  bool update(const sensor_msgs::PointCloud2& input_scan, sensor_msgs::PointCloud2& filtered_scan)
  {
    if(&input_scan == &filtered_scan){
      ROS_ERROR("This filter does not currently support in place copying");
      return false;
    }
    sensor_msgs::PointCloud2 laser_cloud;

    tf::StampedTransform base_link_transform;

    
    try{
      tf_.lookupTransform("/base_link", input_scan.header.frame_id, input_scan.header.stamp, base_link_transform);
    }
    catch(tf::TransformException& ex){
      ROS_ERROR("Transform unavailable %s", ex.what());
      return false;
    }

    
    pcl::PointCloud<PointType>::Ptr pcl_cloud(new pcl::PointCloud<PointType>); 
    pcl::fromROSMsg<PointType>(input_scan, *pcl_cloud);
    
    Eigen::Affine3d base_link_transform_eigen;
    tf::transformTFToEigen(base_link_transform, base_link_transform_eigen);
    
    pcl::PointCloud<PointType>::Ptr pcl_cloud_transformed(new pcl::PointCloud<PointType>); 
    pcl::transformPointCloud(*pcl_cloud, *pcl_cloud_transformed, base_link_transform_eigen);
    
    pcl::toROSMsg<PointType>(*pcl_cloud, filtered_scan);
    filtered_scan.header.frame_id = input_scan.header.frame_id;
    filtered_scan.header.stamp = input_scan.header.stamp;
    
    pcl::PointCloud<PointType>::Ptr pcl_output_cloud(new pcl::PointCloud<PointType>); 
    for ( size_t i = 0; i < pcl_cloud_transformed->points.size(); ++i)
    {
      if ( !inFootprint( pcl_cloud_transformed->points[i] ) )
	pcl_output_cloud->points.push_back(pcl_cloud->points[i]);
    }
    pcl::toROSMsg<PointType>(*pcl_output_cloud, filtered_scan);
    filtered_scan.header.frame_id = input_scan.header.frame_id;
    filtered_scan.header.stamp = input_scan.header.stamp;
  
    return true;
  }


  bool inFootprint(const PointType& point){
    if (filter_by_radius_)
    {
      if(point.x < -1.0 * inscribed_radius_ || point.x > inscribed_radius_ || point.y < -1.0 * inscribed_radius_ || point.y > inscribed_radius_)
	return false;
    }
    else 
    {
      if( point.x < filter_min_x_ || point.x > filter_max_x_ || point.y < filter_min_y_ || point.y > filter_max_y_ || point.z < filter_min_z_ || point.z > filter_max_z_  )
	return false;
    }
      
    return true;
  }

private:
  tf::TransformListener tf_;
  bool filter_by_radius_;
  double inscribed_radius_;
  
  double filter_min_x_;
  double filter_max_x_;
  double filter_min_y_;
  double filter_max_y_;
  double filter_min_z_;
  double filter_max_z_;
  
} ;

}

#endif // POINT_CLOUD_FOOTPRINT_FILTER_H
