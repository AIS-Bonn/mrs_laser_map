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

#include <map_nodelet.h>

#include <random>

#include <pluginlib/class_list_macros.h>
#include <boost/concept_check.hpp>
#include <pcl/common/time.h>
#include <eigen_conversions/eigen_msg.h>

namespace mrs_laser_mapping
{
MapNodelet::MapNodelet()
  : running_(true)
  , first_scan_(true)
  , scan_number_(0)
  
  , map_size_(32)
  , map_levels_(2)
  , map_cell_capacity_(400)
  , map_resolution_(20.0)
  
  , map_downsampled_size_(8)
  , map_downsampled_levels_(4)
  , map_downsampled_cell_capacity_(100)
  , map_downsampled_resolution_(20)

  , scan_assembler_frame_id_("/odom")
  , sensor_frame_id_("laser_scanner_origin")

  , num_scans_registration_(0)
  , num_scans_for_map_publishing_(5)
  , add_new_points_(true)
  , transform_wait_duration_(0.2)
  , multiresolution_map_(new MapType(map_size_, map_resolution_, map_levels_, map_cell_capacity_, map_frame_id_))
  , last_scan_stamp_(0)
  , last_scan_received_stamp_(0)
  , scan_stamp_delta_(0)
  , decrease_rate_(0.08)
  , decrease_once_(false)
  , param_update_occupancy_("occ_update", true)
  , param_clamping_thresh_min_("occ_clamping_thresh_min", -10.f, 0.1f, 10.f, -2.f)
  , param_clamping_thresh_max_("occ_clamping_thresh_max", -10.f, 0.1f, 10.f, 3.5f)
  , param_prob_hit_("occ_prob_hit", -10.f, 0.1f, 10.f, 0.85f)
  , param_prob_miss_("occ_prob_miss", -10.f, 0.1f, 10.f, -0.4f)
  , add_scans_when_torso_rotated_(true)
  , last_torso_yaw_(0.0)
  , scan_buffer_(10)
{
}

MapNodelet::~MapNodelet()
{
  if (running_)
  {
    printf("stopping threads\n");
    fflush(stdout);

    running_ = false;
    tf_broadcaster_thread_->join();
  }

  printf("exiting\n");
  fflush(stdout);
}

void MapNodelet::onInit()
{
  NODELET_INFO("MapNodelet::onInit nodelet...");

  last_base_link_transform_.setIdentity();
  last_base_link_oriented_transform_.setIdentity();

  correction_transform_ = Eigen::Matrix4d::Identity();
  map_orientation_ = Eigen::Matrix4d::Identity();

  ros::NodeHandle& ph = getMTPrivateNodeHandle();
  ros::NodeHandle& phMT = getMTPrivateNodeHandle();

  // get parameters
  ph.getParam("scan_assembler_frame_id", scan_assembler_frame_id_);
  ph.getParam("min_scans_registration", num_scans_registration_);
  ph.getParam("min_scans_for_map_publishing", num_scans_for_map_publishing_);
  ph.getParam("transform_wait_duration", transform_wait_duration_);
  ph.getParam("sensor_frame_id", sensor_frame_id_);

   // parameters for registration
  mrs_laser_maps::RegistrationParameters params;
  ph.param<int>("registration_max_iterations", params.max_iterations_, 100);
  ph.param<double>("registration_prior_prob", params.prior_prob_, 0.25);
  ph.param<double>("registration_soft_assoc_c1", params.soft_assoc_c1_, 0.9);
  ph.param<double>("registration_soft_assoc_c2", params.soft_assoc_c2_, 10.0);
  ph.param<double>("registration_sigma_size_factor", params.sigma_size_factor_, 0.45);
  surfel_registration_.setRegistrationParameters(params);
   
  // parameters for map
  ph.getParam("map_size", map_size_);
  ph.getParam("map_resolution", map_resolution_);
  ph.getParam("map_levels", map_levels_);
  ph.getParam("map_cell_capacity", map_cell_capacity_);

  ph.getParam("map_downsampled_size", map_downsampled_size_);
  ph.getParam("map_downsampled_resolution", map_downsampled_resolution_);
  ph.getParam("map_downsampled_levels", map_downsampled_levels_);
  ph.getParam("map_downsampled_cell_capacity", map_downsampled_cell_capacity_);

  ph.param<bool>("add_scans_when_torso_rotated", add_scans_when_torso_rotated_, true);
  // ph.param<double>("decrease_rate", m_decreaseRate, 0.0);

  multiresolution_map_ = boost::make_shared<MapType>(map_size_, map_resolution_, map_levels_,
                                                                                   map_cell_capacity_, map_frame_id_);
  // subscribe laserscanlines
  sub_cloud_.subscribe(phMT, "input", 3);
  cloud_message_filter_.reset(
      new tf::MessageFilter<sensor_msgs::PointCloud2>(sub_cloud_, tf_listener_, scan_assembler_frame_id_, 100, phMT));
  cloud_message_filter_->registerCallback(boost::bind(&MapNodelet::receivedCloud, this, _1));

  service_clear_map_ = ph.advertiseService("/surfel_map/clear_map", &MapNodelet::clearMapServiceCall, this);
  service_reset_ = ph.advertiseService("/surfel_map/reset", &MapNodelet::resetServiceCall, this);
  service_decrease_once_ = ph.advertiseService("/surfel_map/decrease_once", &MapNodelet::decreaseOnceServiceCall, this);
 
  service_add_points_ =
      ph.advertiseService("/surfel_map/add_points_to_map", &MapNodelet::addPointsToMapServiceCall, this);
  
  tf_broadcaster_thread_ =
      boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&MapNodelet::broadcastTf, this)));
  process_scan_thread_ =
      boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&MapNodelet::processScans, this)));

}

void MapNodelet::broadcastTf()
{
  ros::Rate loopRate(50);
  while (running_)
  {
    updateTransforms(ros::Time::now());
    loopRate.sleep();
  }
}

bool MapNodelet::clearMapServiceCall(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  boost::unique_lock<boost::mutex> lock(mutex_map_);
  clearMap();

  return true;
}

bool MapNodelet::resetServiceCall(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  boost::unique_lock<boost::mutex> lock(mutex_map_);
  clearMap();

  last_base_link_transform_.setIdentity();
  last_base_link_oriented_transform_.setIdentity();

  boost::unique_lock<boost::mutex> correctionLock(mutex_correction_transform_);
  correction_transform_ = Eigen::Matrix4d::Identity();
  map_orientation_ = Eigen::Matrix4d::Identity();

  return true;
}

void MapNodelet::clearMap()
{
  multiresolution_map_ = boost::make_shared<MapType>(map_size_, map_resolution_, map_levels_, map_cell_capacity_, map_frame_id_);

  first_scan_ = true;
  scan_number_ = 0;
  last_scan_stamp_ = ros::Time(0);

  add_new_points_ = true;
}

bool MapNodelet::decreaseOnceServiceCall(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  decrease_once_ = true;

  return true;
}

bool MapNodelet::addPointsToMapServiceCall(AddPointsToMapRequest& req, AddPointsToMapResponse& res)
{
  if (req.addPoints == 0)
  {
    add_new_points_ = false;
  }
  else
  {
    add_new_points_ = true;
  }
  return true;
}

void MapNodelet::receivedCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  NODELET_DEBUG_STREAM("received map from timestamp " << msg->header.stamp << " at time " << ros::Time::now()
                                                      << " diff: " <<  (ros::Time::now() - msg->header.stamp));
  try
  {
    pcl::PointCloud<InputPointType>::Ptr pcl_cloud(new pcl::PointCloud<InputPointType>); // TODO
    pcl::fromROSMsg<InputPointType>(*msg, *pcl_cloud);
    scan_buffer_.push_front(pcl_cloud);
  }
  catch (pcl::PCLException& ex)
  {
    NODELET_ERROR("Failed to convert a message to a pcl type, dropping observation: %s", ex.what());
  }

  if (msg->header.frame_id != scan_assembler_frame_id_)
  {
    NODELET_ERROR_THROTTLE(10.0, "input frame is different from m_scanAssemblerFrameId! This could lead to transform "
                                 "problems.");
  }

  // determine average delta between two scans
  if ( last_scan_received_stamp_ > ros::Time(0) )
  {
    ros::Duration current_scan_stamp_delta (msg->header.stamp - last_scan_received_stamp_);

    if ( scan_stamp_delta_ > ros::Duration(0) )
      scan_stamp_delta_ = current_scan_stamp_delta * 0.1 + scan_stamp_delta_ * 0.9;
    else
      scan_stamp_delta_ = current_scan_stamp_delta;
    NODELET_DEBUG_STREAM("scan stamp delta: " << current_scan_stamp_delta.toSec() << " " << scan_stamp_delta_.toSec());
  }
    
  last_scan_received_stamp_ = msg->header.stamp;
}

void MapNodelet::processScans()
{
  while (running_)
  {
    pcl::PointCloud<InputPointType>::Ptr cloud; 
    
    while (scan_buffer_.size() > 1)
    {
      NODELET_ERROR_STREAM("Too many messages in scan buffer. Dropping scans." );
      scan_buffer_.pop_back(&cloud);
    }

    scan_buffer_.pop_back(&cloud);
    registerScan(cloud);
  }
}

// register the current pointcloud to the one before/the map
void MapNodelet::registerScan(pcl::PointCloud<InputPointType>::Ptr cloud) //TODO
{
  pcl::StopWatch callback_timer;
  callback_timer.reset();

  boost::unique_lock<boost::mutex> lock(mutex_map_);
 
  ros::Time scan_stamp;

  tf::StampedTransform base_link_transform;
  tf::StampedTransform base_link_oriented_transform;
  tf::StampedTransform scan_transform;

  base_link_transform.setIdentity();
  base_link_oriented_transform.setIdentity();
  scan_transform.setIdentity();

  
  NODELET_DEBUG_STREAM("registerScans thread:: received scan from " << cloud->header.stamp << " frame "
                                                                    << cloud->header.frame_id << " with "
                                                                    << cloud->size() << " points");

  scan_stamp = pcl_conversions::fromPCL(cloud->header.stamp);

  if (!first_scan_ && decrease_rate_ > 0.0 && decrease_once_)
  {
    float decrease_per_scan = decrease_rate_ * static_cast<float>((scan_stamp - last_scan_stamp_).toSec());
    multiresolution_map_->lock();
    multiresolution_map_->decreaseAll(decrease_per_scan);
    multiresolution_map_->unlock();
    NODELET_DEBUG_STREAM("decreasing with: " << decrease_per_scan);
    decrease_once_ = false;
  }


  if ( !getTranform(scan_assembler_frame_id_, "/base_link", scan_stamp, base_link_transform) )
    return;

  if (first_scan_)
  {
    // intialize map orientation with orientation between odometry and /base_link
    Eigen::Affine3d odom_rotation;
    tf::transformTFToEigen(tf::Transform(base_link_transform.getRotation().inverse()), odom_rotation);
    map_orientation_ = odom_rotation.matrix();
  }
  else
  {
    // update map orientation with delta between previous and current odometry orientation
    Eigen::Affine3d last_odom_rotation;
    tf::transformTFToEigen(tf::Transform(last_base_link_transform_.getRotation().inverse()), last_odom_rotation);
    Eigen::Affine3d odom_rotation;
    tf::transformTFToEigen(tf::Transform(base_link_transform.getRotation().inverse()), odom_rotation);
    map_orientation_ = (odom_rotation * last_odom_rotation.inverse()).matrix() * map_orientation_;

    // translate map with odometry delta
    tf::Vector3 delta_translation = base_link_transform.getOrigin() - last_base_link_transform_.getOrigin();
    Eigen::Vector3d delta_translation_eigen;
    tf::vectorTFToEigen(delta_translation, delta_translation_eigen);
    multiresolution_map_->lock();
    multiresolution_map_->translateMap(delta_translation_eigen.cast<float>());
    multiresolution_map_->unlock();
    NODELET_DEBUG_STREAM("deltaTranslation: " << delta_translation_eigen.transpose());
  }
  updateTransforms(scan_stamp);

  if ( !getTranform(scan_assembler_frame_id_, map_frame_id_, scan_stamp, base_link_oriented_transform) )
    return;

  Eigen::Affine3d scan_transform_eigen;
  tf::transformTFToEigen(base_link_oriented_transform.inverse(), scan_transform_eigen);

  pcl::PointCloud<InputPointType>::Ptr assembled_cloud(new pcl::PointCloud<InputPointType>());
  
  // transform pointcloud from fixed frame of the scanAssembler to the same frame the map is in (e.g. base_link)
  pcl::transformPointCloud(*cloud, *assembled_cloud, scan_transform_eigen.cast<float>());
  // correct the frame_id
  assembled_cloud->header.frame_id = multiresolution_map_->getFrameId();

  pcl::PointCloud<MapPointType>::Ptr assembled_cloud_rgb(new pcl::PointCloud<MapPointType>()); //TODO
  pcl::copyPointCloud(*assembled_cloud, *assembled_cloud_rgb);
 
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  
  if (!first_scan_ && scan_number_ >= num_scans_registration_)
  {
    pcl::StopWatch registration_timer;
    registration_timer.reset();
    
    MapType mapScene(map_size_, map_resolution_, map_levels_, map_cell_capacity_, map_frame_id_);
   
    mapScene.addCloud(assembled_cloud_rgb);
    mapScene.evaluateAll();

    multiresolution_map_->lock();
    multiresolution_map_->evaluateAll(); // TODO: necessary??
    surfel_registration_.estimateTransformationLevenbergMarquardt(multiresolution_map_.get(), &mapScene, transform );
    multiresolution_map_->unlock();
    
    NODELET_DEBUG_STREAM("estimateTransformationLevenbergMarquardt took: " << registration_timer.getTime() << " ms");
    NODELET_DEBUG_STREAM("transform \n" << transform);
  }
  NODELET_DEBUG_STREAM("after registration: " << callback_timer.getTime() << " ms");

  pcl::PointCloud<MapPointType>::Ptr assembled_cloud_reg(new pcl::PointCloud<MapPointType>); // TODO
  pcl::transformPointCloud(*assembled_cloud_rgb, *assembled_cloud_reg, transform.cast<float>());
  
  if (add_new_points_ && !checkTorsoRotation(scan_stamp))
  {
    multiresolution_map_->setOccupancyParameters(param_clamping_thresh_min_(), param_clamping_thresh_max_(), param_prob_hit_(), param_prob_miss_());
    
    Eigen::Matrix4f sensorTransform = Eigen::Matrix4f::Identity();
    getTranform(sensor_frame_id_, "/base_link", scan_stamp, sensorTransform);

    // add current cloud to map
    multiresolution_map_->addCloud(assembled_cloud_reg, param_update_occupancy_(), sensorTransform.inverse());
  }
  else
  {
    NODELET_DEBUG_STREAM("not adding scan to map");
    if (first_scan_)
    {
      return;
    }
  }
 
 
  NODELET_DEBUG_STREAM("after adding to map: " << callback_timer.getTime() << " ms");

  // translate the map to keep it egocentric
  multiresolution_map_->lock();
  multiresolution_map_->setLastUpdateTimestamp(scan_stamp);
  multiresolution_map_->translateMap(transform.block<3, 1>(0, 3).cast<float>());
  multiresolution_map_->unlock();

  Eigen::Affine3d base_link_transform_eigen;
  tf::transformTFToEigen(base_link_transform, base_link_transform_eigen);
  Eigen::Matrix4d base_link_oriented_in_odom_before_update = base_link_transform_eigen * map_orientation_;  
  
  // update correction transforms for /world_corrected and /base_link_oriented frames
  {
    boost::unique_lock<boost::mutex> lock(mutex_correction_transform_);
    map_orientation_.block<3, 3>(0, 0) = map_orientation_.block<3, 3>(0, 0) * transform.inverse().block<3, 3>(0, 0);
     
    Eigen::Affine3d base_link_oriented_transform_eigen;
    tf::transformTFToEigen(base_link_oriented_transform, base_link_oriented_transform_eigen);

    correction_transform_ *= (base_link_oriented_in_odom_before_update * transform * base_link_oriented_in_odom_before_update.inverse());
    
  }
  updateTransforms(scan_stamp);
  
  // publish point clouds for visualization 
  pcl::PointCloud<pcl::PointXYZ>::Ptr assembled_cloud_reg_XYZ(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*assembled_cloud_reg, *assembled_cloud_reg_XYZ);
  mrs_laser_mapping::SurfelMapPublisher::getInstance()->publishTransformedScenePointCloud(assembled_cloud_reg_XYZ);
  pcl::PointCloud<pcl::PointXYZ>::Ptr assembled_cloud_XYZ(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*assembled_cloud_rgb, *assembled_cloud_XYZ);
  mrs_laser_mapping::SurfelMapPublisher::getInstance()->publishScenePointCloud(assembled_cloud_XYZ);
  
  mrs_laser_mapping::SurfelMapPublisher::getInstance()->publishSurfelMarkers<MapPointType, MapType>(multiresolution_map_);
  mrs_laser_mapping::SurfelMapPublisher::getInstance()->publishPointCloud<MapPointType, MapType>(multiresolution_map_);
  mrs_laser_mapping::SurfelMapPublisher::getInstance()->publishDownsampledPointCloud<MapPointType, MapType>(
      multiresolution_map_, map_downsampled_size_, map_downsampled_resolution_, map_downsampled_levels_, map_downsampled_cell_capacity_);
  mrs_laser_mapping::MapPublisher::getInstance()->publishOccupiedCells<MapPointType, MapType>(multiresolution_map_);
  mrs_laser_mapping::MapPublisher::getInstance()->publishMapLevelColor<MapPointType, MapType>(multiresolution_map_);
  mrs_laser_mapping::MapPublisher::getInstance()->publishMapScanColor<MapPointType, MapType>(multiresolution_map_);
  
  // publish local map for slam nodelet
  if (scan_number_ >= num_scans_for_map_publishing_)
  {
    mrs_laser_mapping::MapPublisher::getInstance()->publishMap<MapPointType, MapType>( multiresolution_map_ );
  }

  NODELET_DEBUG_STREAM("finished processing of scan: " << scan_number_);
  NODELET_DEBUG_STREAM("one callback took: " << callback_timer.getTime() << " ms");
  
  if ( scan_stamp_delta_ > ros::Duration(0) && callback_timer.getTimeSeconds() > scan_stamp_delta_.toSec() )
    NODELET_WARN_STREAM("Processing scan took: " << callback_timer.getTimeSeconds() << "s. Average scan time is " << scan_stamp_delta_.toSec());
  
  scan_number_++;
  last_base_link_transform_ = base_link_transform;
  last_base_link_oriented_transform_ = base_link_oriented_transform;
  last_scan_stamp_ = scan_stamp;
  first_scan_ = false;
  
}

void MapNodelet::updateTransforms(ros::Time time)
{
  boost::unique_lock<boost::mutex> lock(mutex_correction_transform_);
  // update /base_link_oriented
  tf::Transform back_rotation;
  tf::transformEigenToTF(Eigen::Affine3d(map_orientation_), back_rotation);
  tf_broadcaster_.sendTransform(tf::StampedTransform(back_rotation, time, "/base_link", "/base_link_oriented"));
  tf_listener_.setTransform(tf::StampedTransform(back_rotation, time, "/base_link", "/base_link_oriented"));
  
  // update /world_corrected
  tf::StampedTransform transform_tf;
  transform_tf.setIdentity();
  tf::transformEigenToTF(Eigen::Affine3d(correction_transform_).inverse(), transform_tf);
  tf_broadcaster_.sendTransform(
        tf::StampedTransform(transform_tf, time, scan_assembler_frame_id_, "/world_corrected"));
  tf_listener_.setTransform(
      tf::StampedTransform(transform_tf, time, scan_assembler_frame_id_, "/world_corrected"));
}

bool MapNodelet::checkTorsoRotation(ros::Time time)
{
  /*
   * check for torso yaw angle and discard scans when torso is or was rotated (during scan)
   */
  double torso_roll, torso_pitch, torso_yaw;
  bool torso_rotated = false;
  if (!add_scans_when_torso_rotated_)
  {
    try
    {
      tf::StampedTransform torsoTransform;
      if (tf_listener_.waitForTransform("/torso_link", "/base_link", time,
                                        ros::Duration(transform_wait_duration_)))
      {
        tf_listener_.lookupTransform("/torso_link", "/base_link", time, torsoTransform);

        torsoTransform.getBasis().getRPY(torso_roll, torso_pitch, torso_yaw);
      }
      else
      {
        NODELET_ERROR("registerScans:: could not wait for torso transform");
        return false;
      }
    }
    catch (tf::TransformException ex)
    {
      NODELET_ERROR("registerScans:: could not lookup torso transform: %s", ex.what());
      return false;
    }

    // since torso rotation is versy slow we can check the angle at the last and current scan time
    if ((fabs(torso_yaw) > TORSO_YAW_THRESH) || (fabs(last_torso_yaw_) > TORSO_YAW_THRESH))
    {
      torso_rotated = true;
    }
    last_torso_yaw_ = torso_yaw;
  }
  return torso_rotated;
}

bool MapNodelet::getTranform(const std::string& target_frame, const std::string& source_frame, ros::Time time, Eigen::Matrix4f& transform)
{
  tf::StampedTransform transform_tf;
  if ( getTranform(target_frame, source_frame, time, transform_tf) )
  {
    Eigen::Affine3d transform_eigen;
    tf::transformTFToEigen(transform_tf, transform_eigen);
    transform = transform_eigen.matrix().cast<float>();
    return true;
  }
  else
    return false;
}

bool MapNodelet::getTranform(const std::string& target_frame, const std::string& source_frame, ros::Time time, tf::StampedTransform& transform)
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

PLUGINLIB_EXPORT_CLASS(mrs_laser_mapping::MapNodelet, nodelet::Nodelet)
