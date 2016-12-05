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

#include <velodyne_assembler_nodelet.h>

#include <pluginlib/class_list_macros.h>

namespace mrs_laser_mapping
{
VelodyneAssemblerNodelet::VelodyneAssemblerNodelet()
  : frame_id_("/base_link")
  , wait_duration_(0.1)
  , cloud_buffer_(10)
 
  , is_running_(true)
  , is_first_scan_line_(true)
  , is_first_scan_(true)

  , cloud_for_assembler_(new pcl::PointCloud<OutputPointType>())
  
  , last_laser_yaw_angle_(0.0)

  , half_rotation_(false)
  , add_invalid_points_(false)
  , scan_line_number_by_stamp_(false)
{
  NODELET_INFO("Initializing velodyne assembler nodelet.. ");
}

VelodyneAssemblerNodelet::~VelodyneAssemblerNodelet()
{
  is_running_ = false;
  process_clouds_thread_.join();
  NODELET_INFO("Velodyne assembler nodelet exiting normal.. ");
}

void VelodyneAssemblerNodelet::onInit()
{
  NODELET_DEBUG("VelodyneAssemblerNodelet onInit()");

  ros::NodeHandle& ph = getMTPrivateNodeHandle();

  ph.param<std::string>("frame_id", frame_id_, "/base_link");
  ph.param<std::string>("laser_link", laser_link_, "/laser_scanner_center");
  ph.param<bool>("add_invalid_points", add_invalid_points_, false);
  ph.param<bool>("half_rotation", half_rotation_, true);;
  ph.param<bool>("scan_line_number_by_stamp", scan_line_number_by_stamp_, false);
  
  double transform_wait_duration;
  ph.param<double>("transform_wait_duration", transform_wait_duration, 0.1);
  wait_duration_ = ros::Duration(transform_wait_duration);

  // subscribe to point clouds
  sub_clouds_.subscribe(ph, "input", 10);

  message_filter_clouds_.reset(
      new tf::MessageFilter<sensor_msgs::PointCloud2>(sub_clouds_, tf_listener_, frame_id_, 100, ph));
  message_filter_clouds_->registerCallback(boost::bind(&VelodyneAssemblerNodelet::receivedPointCloud, this, _1));

  cloud_publisher_ = ph.advertise<pcl::PointCloud<OutputPointType>>("output", 1);

  process_clouds_thread_ = boost::thread(&VelodyneAssemblerNodelet::processClouds, this);
}

void VelodyneAssemblerNodelet::receivedPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  cloud_buffer_.push_front(*msg);
}

void VelodyneAssemblerNodelet::processClouds()
{
  unsigned int scan_line_number = 0;
  unsigned int point_number = 0;

  while (is_running_)
  {
    if (cloud_buffer_.size() == 0)
    {
      boost::this_thread::sleep(boost::posix_time::milliseconds(1));
      continue;
    }
    NODELET_DEBUG("processing clouds.");

    // get scan line from buffer
    sensor_msgs::PointCloud2 scan_cloud;
    cloud_buffer_.pop_back(&scan_cloud);

    // save frame id from one specific scanner to know when a rotation is complete
    if (is_first_scan_line_ && is_first_scan_)
    {
      scan_header_frame_id_ = scan_cloud.header.frame_id;
    }
    
    // save stamp of first scan line of a scan
    if (is_first_scan_line_)
    {
      first_stamp_ = scan_cloud.header.stamp;
      is_first_scan_line_ = false;
    }
    
    sensor_msgs::PointCloud2 cloud;
    pcl::PointCloud<PointT>::Ptr cloud_transformed(new pcl::PointCloud<PointT>());

    if (!tf_listener_.waitForTransform(frame_id_, scan_cloud.header.frame_id, scan_cloud.header.stamp, wait_duration_))
    {
      NODELET_ERROR_THROTTLE(10.0, "velodyne assembler: could not wait for transform... %f", scan_cloud.header.stamp.toSec());

      // clear buffer if we cannot get the transform. Otherwise, we will never catch up...
      cloud_buffer_.clear();
      continue;
    }

    // transform 2D scan line to 3D point cloud
    try
    {
      sensor_msgs::PointCloud2 cloud_temp;
      pcl_ros::transformPointCloud(frame_id_, scan_cloud, cloud_temp, tf_listener_);
      pcl::fromROSMsg(cloud_temp, *cloud_transformed);
      NODELET_DEBUG("transforme cloud.");
    }
    catch (tf::TransformException& exc)
    {
      NODELET_ERROR_THROTTLE(10.0, "Velodyne Assembler: No transform found");
      NODELET_ERROR_THROTTLE(10.0, "message: '%s'", exc.what());
    }

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_transformed, *cloud_transformed, indices);

    // calculate the scan line number from the scan's timestamp (e.g. when having two lasers)
    if (scan_line_number_by_stamp_)
    {
      scan_line_number = static_cast<unsigned int>(ros::Duration(scan_cloud.header.stamp - first_stamp_).toNSec()*1e-6);
    }
    
    // add meta information
    for (size_t i = 0; i < cloud_transformed->size(); ++i)
    {
      OutputPointType p;
      p.x = cloud_transformed->points[i].x;
      p.y = cloud_transformed->points[i].y;
      p.z = cloud_transformed->points[i].z;
      
      p.scanlineNr = scan_line_number;
      p.pointNr = point_number++;
      
      if (scan_cloud.header.frame_id == scan_header_frame_id_ )
	p.scannerNr = 0;
      else
	p.scannerNr = 1;
      
      cloud_for_assembler_->push_back(p);
    }
    scan_line_number++;

    // get rotation to check if a scan is complete
    tf::StampedTransform laser_rotation_transform;
    if (!tf_listener_.waitForTransform("/base_link", laser_link_, scan_cloud.header.stamp , wait_duration_))
      NODELET_ERROR_THROTTLE(10.0, "velodyne assembler: could not wait for transform... ");

    try
    {
      tf_listener_.lookupTransform("/base_link", laser_link_, scan_cloud.header.stamp, laser_rotation_transform);
    }
    catch (tf::TransformException& exc)
    {
      NODELET_ERROR_THROTTLE(10.0, "Velodyne Assembler: No transform found");
      NODELET_ERROR_THROTTLE(10.0, "message: '%s'", exc.what());
    }

    double laser_yaw = tf::getYaw(laser_rotation_transform.getRotation())+M_PI_2; //add M_PI_2 to have "black spots" on sides
    if (laser_yaw > M_PI)
      laser_yaw = -M_PI + (laser_yaw - M_PI);

    NODELET_DEBUG_STREAM("chekcing if scan is complete." << laser_yaw);
    
    // check for one scanner specified by scan_header_frame_id_ if a rotation is complete
    if (scan_cloud.header.frame_id == scan_header_frame_id_ && isScanComplete(laser_yaw))
    {
      NODELET_DEBUG("scan is complete.");
      
      pcl::PointCloud<OutputPointType>::Ptr output_cloud(new pcl::PointCloud<OutputPointType>());

      cloud_for_assembler_->swap(*output_cloud);
      output_cloud->header.frame_id = frame_id_;
      // adjust stamp to the "mean" stamp
      output_cloud->header.stamp = pcl_conversions::toPCL(first_stamp_ + ((scan_cloud.header.stamp - first_stamp_) * 0.5));
      output_cloud->header.seq = cloud_for_assembler_->header.seq;

      is_first_scan_line_ = true;

      // discard first point cloud. it tends to be incomplete
      if (!is_first_scan_)
      {
        cloud_publisher_.publish(output_cloud);
        NODELET_DEBUG("published cloud");
      }
      else
        is_first_scan_ = false;

      scan_line_number = 0;
    }

    if (scan_cloud.header.frame_id == scan_header_frame_id_)
      last_laser_yaw_angle_ = laser_yaw;
  }
}

bool VelodyneAssemblerNodelet::isScanComplete(float laser_angle)
{ 
  return (last_laser_yaw_angle_ < 0 && laser_angle > 0) || (half_rotation_ && (last_laser_yaw_angle_ > 0 && laser_angle < 0));
}
}

PLUGINLIB_EXPORT_CLASS(mrs_laser_mapping::VelodyneAssemblerNodelet, nodelet::Nodelet)
