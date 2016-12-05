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

#include <slam_nodelet.h>

#include <random>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <eigen_conversions/eigen_msg.h>

#include <pluginlib/class_list_macros.h>

#include <nav_msgs/Odometry.h>

#include <mrs_laser_mapping/graph_publisher.h>



typedef pcl::PointXYZ PointT;


namespace mrs_laser_mapping
{
SlamNodelet::SlamNodelet()
  : is_running_(true)
  , tf_listener_(ros::Duration(20.0))
  , transform_wait_duration_(0.2)
  , pose_is_close_dist_(1.0)
  , pose_is_close_angle_(0.5)
  , pose_is_far_dist_(0.7)
  , add_vertex_by_distance_(true)
  , map_buffer_(5)
{
  last_base_link_oriented_transform_ = Eigen::Matrix4d::Identity();
}

SlamNodelet::~SlamNodelet()
{
  if (is_running_)
  {
    printf("stopping threads\n");
    fflush(stdout);

    is_running_ = false;
  }

  printf("exiting\n");
  fflush(stdout);
}

void SlamNodelet::onInit()
{
  NODELET_INFO("SlamNodelet::onInit nodelet.");

  ros::NodeHandle& ph = getPrivateNodeHandle();
  
  ph.param<std::string>("slam_frame_id", frame_id_slam_map_, "/world_corrected_slam");
  ph.param<std::string>("odom_frame_id", frame_id_odometry_, "/odom");

  
  sub_map_ = ph.subscribe("map", 1, &SlamNodelet::receivedMap, this);

  sub_pose_update_ = ph.subscribe("pose_update", 1, &SlamNodelet::receivedPoseUpdate, this);


  service_add_keyframe_ = ph.advertiseService("add_key_frame", &SlamNodelet::addKeyFrameServiceCall, this);
  service_add_keyframes_by_distance_ =
      ph.advertiseService("add_key_frames_by_distance", &SlamNodelet::addKeyFramesByDistanceServiceCall, this);
  service_clear_slam_graph_ = ph.advertiseService("clear_graph", &SlamNodelet::clearSlamGraphServiceCall, this);
  service_resend_keyframes_ = ph.advertiseService("resend_keyframes", &SlamNodelet::resendKeyframesServiceCall, this);


  ph.getParam("transform_wait_duration", transform_wait_duration_);
      
  // parameters for registration
  mrs_laser_maps::RegistrationParameters params;
  ph.param<int>("max_iter", params.max_iterations_, 100);
  ph.param<double>("prior_prob", params.prior_prob_, 0.25);
  ph.param<double>("soft_assoc_c1", params.soft_assoc_c1_, 0.9);
  ph.param<double>("soft_assoc_c2", params.soft_assoc_c2_, 10.0);
  ph.param<double>("sigma_size_factor", params.sigma_size_factor_, 0.45);
  
  // for adding and connecting edges
  ph.getParam("add_vertex_dist", pose_is_close_dist_);
  ph.getParam("add_vertex_angle", pose_is_close_angle_);
  ph.getParam("connect_vertex_dist", pose_is_far_dist_);
  ph.param("add_vertex_by_distance", add_vertex_by_distance_, true);

  initSLAMGraph();
  slam_->setRegistrationParameters(params);

  tf_broadcaster_thread_ =
      boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&SlamNodelet::broadcastTf, this)));
  process_map_thread_ =
      boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&SlamNodelet::processMapQueue, this)));
}

void SlamNodelet::initSLAMGraph()
{
  NODELET_DEBUG("SlamNodelet::initSLAMGraph()");

  first_map_ = true;

  slam_ = boost::make_shared<SlamGraph>();

  slam_->pose_is_close_dist_ = pose_is_close_dist_;
  slam_->pose_is_close_angle_ = pose_is_close_angle_;
  slam_->pose_is_far_dist_ = pose_is_far_dist_;

  slam_->add_nodes_by_distance_ = add_vertex_by_distance_;

  transform_world_corrected_slam_.setIdentity();
}

void SlamNodelet::broadcastTf()
{
  ros::Rate loopRate(50);
  while (is_running_)
  {
    {
      boost::unique_lock<boost::mutex> lock(correction_transform_mutex_);
      // publish current slam pose
      tf_broadcaster_.sendTransform(
          tf::StampedTransform(transform_world_corrected_slam_, ros::Time::now(), frame_id_odometry_, frame_id_slam_map_));
      tf_listener_.setTransform(
          tf::StampedTransform(transform_world_corrected_slam_, ros::Time::now(), frame_id_odometry_, frame_id_slam_map_));
    }
    loopRate.sleep();
  }
}

bool SlamNodelet::addKeyFrameServiceCall(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  slam_->add_node_manual_ = true;
  return true;
}

bool SlamNodelet::addKeyFramesByDistanceServiceCall(mrs_laser_mapping::AddKeyFramesByDistance::Request& req,
                                                    mrs_laser_mapping::AddKeyFramesByDistance::Response& res)
{
  if (req.addKeyFrames == 0)
  {
    add_vertex_by_distance_ = false;
  }
  else
  {
    add_vertex_by_distance_ = true;
  }
  slam_->add_nodes_by_distance_ = add_vertex_by_distance_;
  return true;
}

bool SlamNodelet::clearSlamGraphServiceCall(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  initSLAMGraph();
  return true;
}

bool SlamNodelet::resendKeyframesServiceCall(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  mrs_laser_mapping::GraphPublisher<MapPointType, MapType>::getInstance()->resetNodeCounter();
  return true;
}

void SlamNodelet::receivedPoseUpdate(const geometry_msgs::PoseStamped& msg)
{
  NODELET_INFO("SlamNodelet::receivedPoseUpdate");

  Eigen::Matrix4d pose;

  pose(0, 3) = msg.pose.position.x;
  pose(1, 3) = msg.pose.position.y;
  pose(2, 3) = msg.pose.position.z;

  Eigen::Quaterniond q(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);

  pose.block<3, 3>(0, 0) = q.matrix();

  NODELET_INFO_STREAM("pose: " << pose << " header " << msg.header.frame_id);

  slam_->setPose(pose);
}

void SlamNodelet::processMapQueue()
{
  while (is_running_)
  {
    mrs_laser_mapping::MultiResolutionMapMsgConstPtr map_msg;
    
    while (map_buffer_.size() > 1)	
    {
      NODELET_WARN_STREAM("SlamNodelet: too many messages in buffer. Dropping messages.");
      map_buffer_.pop_back(&map_msg);  
    }
    map_buffer_.pop_back(&map_msg);
    update(map_msg);
  }
}

void SlamNodelet::receivedMap(const mrs_laser_mapping::MultiResolutionMapMsgConstPtr& msg)
{
//   pcl::StopWatch watch;
  NODELET_DEBUG_STREAM("received map from timestamp " << msg->header.stamp << " at time " << ros::Time::now());
  
//   ros::Duration map_time_diff = ros::Time::now() - msg->header.stamp;
// 
//   if (map_time_diff.toSec() > 7.0)
//   {
//     NODELET_ERROR_STREAM("Skipping old map. Latency: " << map_time_diff);
//     return;
//   }
//   else if (map_time_diff.toSec() > 4.0)
//     NODELET_WARN_STREAM("Processing old map. Latency: " << map_time_diff);
/*
    NODELET_INFO_STREAM("creating map with : " << msg->size_in_meters << " - " <<  msg->resolution << " " << msg->levels << " " << msg->cell_capacity << " " << msg->header.frame_id);

  MapType::Ptr map = boost::make_shared<MapType>(msg->size_in_meters, msg->resolution, msg->levels, msg->cell_capacity, msg->header.frame_id);

  tbb::parallel_for(size_t(0), static_cast<size_t>(msg->levels), size_t(1) , [=](size_t i) 
    {  
      
      pcl::PointCloud<MapPointType>::Ptr points(new pcl::PointCloud<MapPointType>());
      pcl::fromROSMsg(msg->cloud[i], *points);
      map->setLevel(points, i);
      
    });

//   for (unsigned int i = 0; i < msg->levels; i++)
//   {
//     pcl::PointCloud<MapPointType>::Ptr points(new pcl::PointCloud<MapPointType>());
//     pcl::fromROSMsg(msg->cloud[i], *points);
//     map->setLevel(points, i);
//   }
  
  map->setLastUpdateTimestamp(msg->header.stamp);
  map->evaluateAll();*/
  
  map_buffer_.push_front(msg);

}

void SlamNodelet::update(const mrs_laser_mapping::MultiResolutionMapMsgConstPtr& msg)
{ 
  pcl::StopWatch update_watch;
  NODELET_DEBUG_STREAM("creating map with : " << msg->size_in_meters << " - " <<  msg->resolution << " " << msg->levels << " " << msg->cell_capacity << " " << msg->header.frame_id);
  MapType::Ptr map = boost::make_shared<MapType>(msg->size_in_meters, msg->resolution, msg->levels, msg->cell_capacity, msg->header.frame_id);

  bool map_distorted = msg->distorted;
  
  tbb::parallel_for(size_t(0), static_cast<size_t>(msg->levels), size_t(1) , [=](size_t i) 
  {      
    pcl::PointCloud<MapPointType>::Ptr points(new pcl::PointCloud<MapPointType>());
    pcl::fromROSMsg(msg->cloud[i], *points);
    map->setLevel(points, i);
    
  });

  map->setLastUpdateTimestamp(msg->header.stamp);
  map->evaluateAll();
  
  ros::Time map_time = map->getLastUpdateTimestamp();
  std::string map_frame = map->getFrameId();
  
  Eigen::Matrix4d odom_to_blo_transform;
  if (!getTranform(frame_id_odometry_, map_frame, map_time, odom_to_blo_transform ))
    return;

  Eigen::Matrix4d world_corrected_to_blo_transform;
  if (!getTranform("/world_corrected", map_frame, map_time, world_corrected_to_blo_transform))
    return;

  Eigen::Matrix4d blo_delta_transform =
      last_base_link_oriented_transform_.inverse() * world_corrected_to_blo_transform;

  if (first_map_)
  {
    first_map_ = false;
    
    // initialize correction transform by correction from local mapping 
    boost::unique_lock<boost::mutex> lock(correction_transform_mutex_);
    tf::transformEigenToTF(Eigen::Affine3d((world_corrected_to_blo_transform * odom_to_blo_transform.inverse())).inverse(),
                           transform_world_corrected_slam_);
  }
  else
  {
    // get current base_link_oriented transform from the slam frame at last scans time
    Eigen::Matrix4d slam_frame_to_blo_transform;
    if (!getTranform(frame_id_slam_map_, map_frame, map_time, slam_frame_to_blo_transform ))
      return;
    
    // update tracked pose by registration result from the local mapping
    g2o::VertexSE3* v_ref = dynamic_cast<g2o::VertexSE3*>(
        slam_->optimizer_->vertex(slam_->graph_nodes_[slam_->reference_node_id_]->node_id_));
    Eigen::Matrix4d reference_vertex_transform = v_ref->estimate().matrix();
    Eigen::Matrix4d reference_vertex_in_blo_transform = slam_frame_to_blo_transform.inverse() * reference_vertex_transform;
    slam_->last_transform_ = reference_vertex_in_blo_transform.inverse() * blo_delta_transform *
                             reference_vertex_in_blo_transform * slam_->last_transform_;
  }

  last_base_link_oriented_transform_ = world_corrected_to_blo_transform;

  // update the slam graph and track the pose
  slam_->update( map, map_distorted );
  
  NODELET_DEBUG_STREAM("update(): after updating slam graph: " << update_watch.getTime());
  mrs_laser_mapping::GraphPublisher<MapPointType, MapType>::getInstance()->publishGraph(slam_);
  NODELET_DEBUG_STREAM("update(): after publishing graph: " << update_watch.getTime());
  
  last_update_time_ = map_time;

  // get current slam pose to broadcast /world_corrected_slam
  g2o::VertexSE3* tracking_vertex = dynamic_cast<g2o::VertexSE3*>(
      slam_->optimizer_->vertex(slam_->graph_nodes_[slam_->reference_node_id_]->node_id_));
  Eigen::Matrix4d tracking_vertex_transform = tracking_vertex->estimate().matrix();
  Eigen::Matrix4d tracked_pose_transform = tracking_vertex_transform * slam_->last_transform_;

  {
    // update m_worldCorrectedSlam by current slam pose and odometry (since published in odom frame)
    boost::unique_lock<boost::mutex> lock(correction_transform_mutex_);
    tf::transformEigenToTF(Eigen::Affine3d((tracked_pose_transform * odom_to_blo_transform.inverse())).inverse(),
                           transform_world_corrected_slam_);
  }
  NODELET_DEBUG_STREAM("finished slam procesing from timestamp " << map_time << " at time "
                                                                 << ros::Time::now());

  
  // publish odometry message
  tf::StampedTransform base_link_in_slam_frame_transform;
  getTranform(frame_id_slam_map_, "/base_link", map_time, base_link_in_slam_frame_transform);
    
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = map_time;
  odom_msg.header.frame_id = frame_id_slam_map_;
  tf::poseTFToMsg(base_link_in_slam_frame_transform, odom_msg.pose.pose);
//   mrs_laser_mapping::GraphPublisher<MapPointType, MapType>::getInstance()->publishOdometryGraph( slam_, odom_msg);
  NODELET_DEBUG_STREAM("update(): end of update: " << update_watch.getTime());
}


bool SlamNodelet::getTranform(const std::string& target_frame, const std::string& source_frame, ros::Time time, Eigen::Matrix4d& transform)
{
  tf::StampedTransform transform_tf;
  if ( getTranform(target_frame, source_frame, time, transform_tf) )
  {
    Eigen::Affine3d transform_eigen;
    tf::transformTFToEigen(transform_tf, transform_eigen);
    transform = transform_eigen.matrix();
    return true;
  }
  else
    return false;
}

bool SlamNodelet::getTranform(const std::string& target_frame, const std::string& source_frame, ros::Time time, tf::StampedTransform& transform)
{
  try
  {
    
    if (tf_listener_.waitForTransform(target_frame, source_frame, time,
                                      ros::Duration(transform_wait_duration_)))
    {
      tf_listener_.lookupTransform(target_frame, source_frame, time, transform);
    }
    else
    {
      NODELET_ERROR_STREAM("getTranform: could not wait for transform. target_frame: " << target_frame << " source_frame: " << source_frame );
      return false;
    }
  }
  catch (tf::TransformException ex)
  {
    NODELET_ERROR("getTranform: could not lookup transform: %s", ex.what());
    return false;
  }
  return true;
}


}
PLUGINLIB_EXPORT_CLASS(mrs_laser_mapping::SlamNodelet, nodelet::Nodelet)
