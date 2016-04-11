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

#ifndef _MAP_NODELET_H_
#define _MAP_NODELET_H_

#include <string>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <message_filters/subscriber.h>
#include <tf/tfMessage.h>
#include <tf/message_filter.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>

#include <config_server/parameter.h>

#include <mrs_laser_maps/map_point_types.h>
#include <mrs_laser_maps/map_multiresolution.h>
#include <mrs_laser_maps/multiresolution_surfel_registration.h>
#include <mrs_laser_maps/synchronized_circular_buffer.h>
#include <mrs_laser_mapping/trajectory_publisher.h>
#include <mrs_laser_mapping/map_publisher.h>
#include <mrs_laser_mapping/surfelmap_publisher.h>
#include <mrs_laser_mapping/AddPointsToMap.h>

namespace mrs_laser_mapping
{

// typedef mrs_laser_maps::MultiResolutionalMap<PointXYZRGBScanLabel> mrs_laser_maps::MapType;

class MapNodelet : public nodelet::Nodelet
{
public:
  MapNodelet();
  virtual ~MapNodelet();
  virtual void onInit();
	
  bool clearMapServiceCall(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool resetServiceCall(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool decreaseOnceServiceCall(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool addPointsToMapServiceCall(AddPointsToMapRequest& req, AddPointsToMapResponse& res);
  bool storeMapServiceCall(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool restoreMapServiceCall(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  void receivedCloud(const sensor_msgs::PointCloud2ConstPtr& msg);

protected:
  void processScans();
	
  void registerScan(pcl::PointCloud<mrs_laser_maps::InputPointType>::Ptr cloud); // TODO
	
  void broadcastTf();
	
	void updateBaseLinkOrientedTransform(ros::Time time);
	
	bool checkTorsoRotation(ros::Time time);
	
	bool getTranform(const std::string& target_frame, const std::string& source_frame, ros::Time time, Eigen::Matrix4f& transform);
	bool getTranform(const std::string& target_frame, const std::string& source_frame, ros::Time time, tf::StampedTransform& transform);
	
  void clearMap();

private:
	// TF synchronized subscriber for laser scans
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_;
  boost::shared_ptr<tf::MessageFilter<sensor_msgs::PointCloud2>> cloud_message_filter_;
	
	ros::ServiceServer service_clear_map_;
  ros::ServiceServer service_reset_;
  ros::ServiceServer service_decrease_once_;
  ros::ServiceServer service_add_points_;
  ros::ServiceServer service_store_map_;
  ros::ServiceServer service_restore_map_;
	
	tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
	
	volatile bool running_;
  bool first_scan_;
  int scan_number_;
	
  int map_size_;
  int map_levels_;
  int map_cell_capacity_;
  double map_resolution_;

  int map_downsampled_size_;
  int map_downsampled_levels_;
  int map_downsampled_cell_capacity_;
  double map_downsampled_resolution_;

	// parameters for registration
  double registration_prior_prob_;
	double registration_sigma_size_factor_;
  double registration_soft_assoc_c1_;
  int registration_soft_assoc_c2_;
  int registration_max_iterations_;

  std::string scan_assembler_frame_id_;
  const std::string map_frame_id_ = "base_link_oriented";
  std::string sensor_frame_id_;

  int num_scans_registration_;
  int num_scans_for_map_publishing_;
	
  bool add_new_points_;
  double transform_wait_duration_;

  boost::shared_ptr<mrs_laser_maps::MapType> multiresolution_map_;
	
  boost::mutex mutex_map_;
  boost::mutex mutex_correction_transform_;
  
	Eigen::Matrix4d correction_transform_;
	Eigen::Matrix4d map_orientation_;
 
	tf::StampedTransform last_base_link_transform_;
  tf::StampedTransform last_base_link_oriented_transform_;

	ros::Time last_scan_stamp_;
	
	double decrease_rate_;
  bool decrease_once_;
	
  config_server::Parameter<bool> param_update_occupancy_;
  config_server::Parameter<float> param_clamping_thresh_min_;
  config_server::Parameter<float> param_clamping_thresh_max_;
  config_server::Parameter<float> param_prob_hit_;
  config_server::Parameter<float> param_prob_miss_;

  bool add_scans_when_torso_rotated_;
  double last_torso_yaw_;
  double const TORSO_YAW_THRESH = (M_PI / 180.0);

  mrs_laser_maps::synchronized_circular_buffer<pcl::PointCloud<mrs_laser_maps::InputPointType>::Ptr> scan_buffer_; 
  boost::shared_ptr<boost::thread> process_scan_thread_;
  boost::shared_ptr<boost::thread> tf_broadcaster_thread_;

  mrs_laser_maps::MultiResolutionSurfelRegistration surfel_registration_;
	
	struct MapSnapshot
  {
    MapSnapshot(boost::shared_ptr<mrs_laser_maps::MapType> map, Eigen::Matrix4d map_orientation, Eigen::Matrix4d correction_transform)
      : multiresolution_map_(map), map_orientation_(map_orientation), correction_transform_(correction_transform)
    {
    }
    boost::shared_ptr<mrs_laser_maps::MapType> multiresolution_map_;
    Eigen::Matrix4d map_orientation_;
    Eigen::Matrix4d correction_transform_;
  };
  boost::circular_buffer<boost::shared_ptr<mrs_laser_mapping::MapNodelet::MapSnapshot>> map_history_;

};
}

#endif
