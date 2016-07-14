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

#include <scan_assembler_nodelet.h>

#include <pluginlib/class_list_macros.h>

namespace mrs_laser_mapping
{
ScanAssemblerNodelet::ScanAssemblerNodelet()
  : frame_id_("/base_link")
  , wait_duration_(0.1)
  , scan_line_buffer_(40)
 
  , is_running_(true)
  , is_first_scan_line_(true)
  , is_first_scan_(true)

  , cloud_for_assembler_(new pcl::PointCloud<OutputPointType>())
  
  , last_laser_yaw_angle_(0.0)

  , half_rotation_(false)
  , add_invalid_points_(false)
  , scan_line_number_by_stamp_(false)
{
  NODELET_INFO("Initializing scan assembler nodelet.. ");
}

ScanAssemblerNodelet::~ScanAssemblerNodelet()
{
  is_running_ = false;
  process_scan_thread_.join();
  NODELET_INFO("Scan assembler nodelet exiting normal.. ");
}

void ScanAssemblerNodelet::onInit()
{
  NODELET_DEBUG("ScanAssemblerNodelet onInit()");

  ros::NodeHandle& ph = getMTPrivateNodeHandle();

  ph.param<std::string>("frame_id", frame_id_, "/base_link");
  ph.param<bool>("add_invalid_points", add_invalid_points_, false);
  ph.param<bool>("half_rotation", half_rotation_, true);;
  ph.param<bool>("scan_line_number_by_stamp", scan_line_number_by_stamp_, false);
  
  double transform_wait_duration;
  ph.param<double>("transform_wait_duration", transform_wait_duration, 0.1);
  wait_duration_ = ros::Duration(transform_wait_duration);

  // subscribe laserscanlines
  sub_laser_scan_.subscribe(ph, "input", 10);

  message_filter_laser_scan_.reset(
      new tf::MessageFilter<sensor_msgs::LaserScan>(sub_laser_scan_, tf_listener_, frame_id_, 100, ph));
  message_filter_laser_scan_->registerCallback(boost::bind(&ScanAssemblerNodelet::receivedLaserScan, this, _1));

  scan_publisher_ = ph.advertise<pcl::PointCloud<OutputPointType>>("output", 1);

  process_scan_thread_ = boost::thread(&ScanAssemblerNodelet::processScans, this);
}

void ScanAssemblerNodelet::receivedLaserScan(const sensor_msgs::LaserScanConstPtr& msg)
{
  scan_line_buffer_.push_front(*msg);
}

void ScanAssemblerNodelet::processScans()
{
  unsigned int scan_line_number = 0;
  unsigned int point_number = 0;

  while (is_running_)
  {
    if (scan_line_buffer_.size() == 0)
    {
      boost::this_thread::sleep(boost::posix_time::milliseconds(1));
      continue;
    }
    NODELET_DEBUG("processing scans.");

    // get scan line from buffer
    sensor_msgs::LaserScan scan;
    scan_line_buffer_.pop_back(&scan);

    // save frame id from one specific scanner to know when a rotation is complete
    if (is_first_scan_line_ && is_first_scan_)
    {
      scan_header_frame_id_ = scan.header.frame_id;
    }
    
    // save stamp of first scan line of a scan
    if (is_first_scan_line_)
    {
      first_stamp_ = scan.header.stamp;
      is_first_scan_line_ = false;
    }
    
    sensor_msgs::PointCloud2 cloud;
    pcl::PointCloud<PointT>::Ptr cloud_transformed(new pcl::PointCloud<PointT>());

    if (!tf_listener_.waitForTransform(frame_id_, scan.header.frame_id, scan.header.stamp +
		ros::Duration().fromSec((scan.ranges.size()) * scan.time_increment), wait_duration_))
    {
      NODELET_ERROR_THROTTLE(10.0, "scan assembler: could not wait for transform... %f %f", scan.header.stamp.toSec(),
	  (scan.header.stamp + ros::Duration().fromSec((scan.ranges.size()) * scan.time_increment)).toSec());

      // clear buffer if we cannot get the transform. Otherwise, we will never catch up...
      scan_line_buffer_.clear();
      continue;
    }

    // transform 2D scan line to 3D point cloud
    try
    {
      // keep points with max range measurements
      if (add_invalid_points_)
      {
        for (unsigned int i = 0; i < scan.ranges.size(); i++)
        {
          if (scan.ranges[i] >= scan.range_max)
          {
            scan.ranges[i] = scan.range_max - 0.1;
          }
        }
        scan_projector_.transformLaserScanToPointCloud(frame_id_, scan, cloud, tf_listener_, -1.0,
        laser_geometry::channel_option::Default);
      }
      else
      {
        scan_projector_.transformLaserScanToPointCloud(frame_id_, scan, cloud, tf_listener_, scan.range_max - 0.01,
        laser_geometry::channel_option::Default);
      }

      // fix fields.count member
      for (unsigned int i = 0; i < cloud.fields.size(); i++)
        cloud.fields[i].count = 1;
      
      pcl::fromROSMsg(cloud, *cloud_transformed);
    }
    catch (tf::TransformException& exc)
    {
      NODELET_ERROR_THROTTLE(10.0, "Scan Assembler: No transform found");
      NODELET_ERROR_THROTTLE(10.0, "message: '%s'", exc.what());
    }

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_transformed, *cloud_transformed, indices);

    // calculate the scan line number from the scan's timestamp (e.g. when having two lasers)
    if (scan_line_number_by_stamp_)
    {
      scan_line_number = static_cast<unsigned int>(ros::Duration(scan.header.stamp - first_stamp_).toNSec()*1e-6);
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
      
      cloud_for_assembler_->push_back(p);
    }
    scan_line_number++;

    // get rotation to check if a scan is complete
    tf::StampedTransform laser_rotation_transform;
    if (!tf_listener_.waitForTransform("/base_link", scan_header_frame_id_,
	    scan.header.stamp + ros::Duration().fromSec((scan.ranges.size()) * scan.time_increment), wait_duration_))
      NODELET_ERROR_THROTTLE(10.0, "scan assembler: could not wait for transform... %f %f", scan.header.stamp.toSec(),
	  (scan.header.stamp + ros::Duration().fromSec((scan.ranges.size()) * scan.time_increment)).toSec());

    try
    {
      tf_listener_.lookupTransform("/base_link", scan_header_frame_id_, scan.header.stamp +
					ros::Duration().fromSec((scan.ranges.size()) * scan.time_increment), laser_rotation_transform);
    }
    catch (tf::TransformException& exc)
    {
      NODELET_ERROR_THROTTLE(10.0, "Scan Assembler: No transform found");
      NODELET_ERROR_THROTTLE(10.0, "message: '%s'", exc.what());
    }

    double laser_yaw = tf::getYaw(laser_rotation_transform.getRotation()); //add M_PI_2 to have "black spots" on sides
    if (laser_yaw > M_PI)
      laser_yaw = -M_PI + (laser_yaw - M_PI);

    // check for one scanner specified by scan_header_frame_id_ if a rotation is complete
    if (scan.header.frame_id == scan_header_frame_id_ && isScanComplete(laser_yaw))
    {
      pcl::PointCloud<OutputPointType>::Ptr output_cloud(new pcl::PointCloud<OutputPointType>());

      cloud_for_assembler_->swap(*output_cloud);
      output_cloud->header.frame_id = frame_id_;
      // adjust stamp to the "mean" stamp
      output_cloud->header.stamp = pcl_conversions::toPCL(first_stamp_ + ((scan.header.stamp - first_stamp_) * 0.5));
      output_cloud->header.seq = cloud_for_assembler_->header.seq;

      is_first_scan_line_ = true;

      // discard first point cloud. it tends to be incomplete
      if (!is_first_scan_)
      {
        scan_publisher_.publish(output_cloud);
        NODELET_DEBUG("published cloud");
      }
      else
        is_first_scan_ = false;

      scan_line_number = 0;
    }

    if (scan.header.frame_id == scan_header_frame_id_)
      last_laser_yaw_angle_ = laser_yaw;
  }
}

bool ScanAssemblerNodelet::isScanComplete(float laser_angle)
{ 
  return (last_laser_yaw_angle_ < 0 && laser_angle > 0) || (half_rotation_ && (last_laser_yaw_angle_ > 0 && laser_angle < 0));
}
}

PLUGINLIB_EXPORT_CLASS(mrs_laser_mapping::ScanAssemblerNodelet, nodelet::Nodelet)
