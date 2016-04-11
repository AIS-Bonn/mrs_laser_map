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

#ifndef _SCAN_ASSEMBLER_NODELET_H_
#define _SCAN_ASSEMBLER_NODELET_H_

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <tf/message_filter.h>
#include <tf/tfMessage.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>

#include <mrs_laser_maps/map_point_types.h>
#include <mrs_laser_maps/synchronized_circular_buffer.h>

namespace mrs_laser_mapping
{
class ScanAssemblerNodelet : public nodelet::Nodelet
{
public:
  typedef pcl::PointXYZ PointT;
  typedef PointXYZScanLine OutputPointType;
  ScanAssemblerNodelet();
  virtual ~ScanAssemblerNodelet();
  virtual void onInit();
  void receivedLaserScan(const sensor_msgs::LaserScanConstPtr& msg);

protected:
  void processScans();
  bool isScanComplete(pcl::PointCloud<PointT>::Ptr scan_cloud);
  bool isScanComplete(float laser_angle);

private:
	// a fixed frame (something like the world frame), transforms between this frame and the base_link are important
  std::string frame_id_;
	
	ros::Duration wait_duration_;

	mrs_laser_maps::synchronized_circular_buffer<sensor_msgs::LaserScan> scan_line_buffer_;

  bool is_running_;
  bool is_first_scan_line_;
  bool is_first_scan_;
	
  boost::thread process_scan_thread_;

  ros::Publisher scan_publisher_;
  ros::Time first_stamp_;

  tf::TransformListener tf_listener_;

  // TF synchronized subscriber for laser scans
  message_filters::Subscriber<sensor_msgs::LaserScan> sub_laser_scan_;
  boost::shared_ptr<tf::MessageFilter<sensor_msgs::LaserScan>> message_filter_laser_scan_;

  laser_geometry::LaserProjection scan_projector_;
  pcl::PointCloud<OutputPointType>::Ptr cloud_for_assembler_;

  double last_laser_yaw_angle_;

  std::string scan_header_frame_id_;
  
  bool half_scan_;
  bool use_invalid_points_;
};
}

#endif
