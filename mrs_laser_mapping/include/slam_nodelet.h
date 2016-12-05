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

#ifndef _SLAM_NODELET_H_
#define _SLAM_NODELET_H_

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <message_filters/subscriber.h>

#include <tf/tfMessage.h>
#include <tf/message_filter.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <config_server/parameter.h>

#include <mrs_laser_maps/synchronized_circular_buffer.h>
#include <mrs_laser_maps/multiresolution_surfel_registration.h>
#include <mrs_laser_maps/map_multiresolution.h>
#include <mrs_laser_mapping/slam_graph.h>
#include <mrs_laser_maps/map_point_types.h>

#include <std_srvs/Empty.h>
#include <mrs_laser_mapping/MultiResolutionMapMsg.h>
#include <mrs_laser_mapping/AddKeyFramesByDistance.h>


namespace mrs_laser_mapping
{
class SlamNodelet : public nodelet::Nodelet
{
public:
  typedef PointXYZRGBScanLabel MapPointType;
  typedef mrs_laser_maps::MultiResolutionalMap<MapPointType> MapType;
  typedef SlamGraph::Ptr GraphPtr;

  SlamNodelet();
  virtual ~SlamNodelet();
  virtual void onInit();

  void initSLAMGraph();

  bool addKeyFrameServiceCall(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool addKeyFramesByDistanceServiceCall(mrs_laser_mapping::AddKeyFramesByDistance::Request& req,
                                         mrs_laser_mapping::AddKeyFramesByDistance::Response& res);
  bool clearSlamGraphServiceCall(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool resendKeyframesServiceCall(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  void processMapQueue();
  void receivedMap(const mrs_laser_mapping::MultiResolutionMapMsgConstPtr& msg);
  void update(const mrs_laser_mapping::MultiResolutionMapMsgConstPtr& msg);

  void receivedPoseUpdate(const geometry_msgs::PoseStamped& msg);

  void broadcastTf();

  bool getTranform(const std::string& target_frame, const std::string& source_frame, ros::Time time, Eigen::Matrix4d& transform);
  bool getTranform(const std::string& target_frame, const std::string& source_frame, ros::Time time, tf::StampedTransform& transform);
  
protected:


private:
  volatile bool is_running_;

  boost::shared_ptr<boost::thread> tf_broadcaster_thread_;
  boost::mutex correction_transform_mutex_;

  ros::Subscriber sub_map_;
  ros::Subscriber sub_pose_update_;

  ros::ServiceServer service_clear_slam_graph_;
  ros::ServiceServer service_add_keyframe_;
  ros::ServiceServer service_add_keyframes_by_distance_;
  ros::ServiceServer service_resend_keyframes_;

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;

  double transform_wait_duration_;
  
  Eigen::Matrix4d last_base_link_oriented_transform_;
  tf::StampedTransform transform_world_corrected_slam_;

  std::string frame_id_slam_map_;
  std::string frame_id_odometry_;

  GraphPtr slam_;

  double pose_is_close_dist_;
  double pose_is_close_angle_;

  double pose_is_far_dist_;

  bool first_map_;
  bool add_vertex_by_distance_;

  ros::Time last_update_time_;

  mrs_laser_maps::synchronized_circular_buffer<mrs_laser_mapping::MultiResolutionMapMsgConstPtr> map_buffer_;
  boost::mutex map_buffer_mutex_;
  boost::shared_ptr<boost::thread> process_map_thread_;
};
}

#endif
