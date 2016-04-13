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

typedef pcl::PointXYZ PointT;

namespace
{
class VectorStreamBuf : public std::streambuf
{
public:
  explicit VectorStreamBuf(std::vector<uint8_t>* out) : m_out(out)
  {
  }

  virtual int overflow(int c)
  {
    m_out->push_back(c);
    return 0;
  }

private:
  std::vector<uint8_t>* m_out;
};

class VectorStream : public std::ostream
{
public:
  explicit VectorStream(std::vector<uint8_t>* out) : m_buf(out)
  {
    rdbuf(&m_buf);
  }

private:
  VectorStreamBuf m_buf;
};
}

namespace mrs_laser_mapping
{
SlamNodelet::SlamNodelet()
  : m_running(true)
  , tf_listener_(ros::Duration(20.0))
  , transform_wait_duration_(0.2)
  , m_slamMapFrameId("/world_corrected_slam")
  , m_heightImageFrame("/map")
  , m_odometryFrameId("/odom")
  , m_maxIter(100)
  , m_priorProb(0.25)
  , m_softAssocC1(0.9)
  , m_softAssocC2(10)
  , m_sigmaSizeFactor(0.45)
  , m_poseIsCloseDist(1.0)
  , m_poseIsCloseAngle(0.5)
  , m_poseIsFarDist(0.7)
  , m_heightImageSizeX()
  , m_heightImageSizeY()
  , m_heightImageResolution()
  , m_addKeyFramesByDistance(true)
  , m_keyFrameCounter(0)
  , m_mapBuffer(5)
  , m_evaluatePointDrift(false)
{
}

SlamNodelet::~SlamNodelet()
{
  if (m_running)
  {
    printf("stopping threads\n");
    fflush(stdout);

    m_running = false;
  }

  printf("exiting\n");
  fflush(stdout);
}

void SlamNodelet::onInit()
{
  NODELET_INFO("SlamNodelet::onInit nodelet...");

  ros::NodeHandle& ph = getMTPrivateNodeHandle();

  ph.getParam("slam_frame_id", m_slamMapFrameId);
  ph.getParam("odom_frame_id", m_odometryFrameId);

  
  m_mapSubscriber = ph.subscribe("map", 1, &SlamNodelet::receivedMap, this);

  m_poseUpdateSubscriber = ph.subscribe("pose_update", 1, &SlamNodelet::receivedPoseUpdate, this);

  m_slamGraphMarkerPublisher = ph.advertise<visualization_msgs::Marker>("slam_graph_marker", 10);
  m_mapPublisher = ph.advertise<sensor_msgs::PointCloud2>("slam_map", 10);
  m_mapDownsampledPublisher = ph.advertise<sensor_msgs::PointCloud2>("slam_map_downsampled", 10);
  m_referenceKeyframePublisher = ph.advertise<sensor_msgs::PointCloud2>("slam_reference_keyframe", 10);
  // 		m_pointCloudGlobal = ph.advertise<sensor_msgs::PointCloud2>("global_map", 10);
  m_heightImagePublisher = ph.advertise<sensor_msgs::Image>("height", 1);
  m_keyFramePublisher = ph.advertise<mrs_laser_mapping::KeyFrame>("keyframes", 1);
  m_keyFrameTransformsPublisher = ph.advertise<mrs_laser_mapping::KeyFrameTransforms>("keyframe_transforms", 1);
  m_odometryPublisher = ph.advertise<nav_msgs::Odometry>("odometry", 1);
  m_localMapPublisher = ph.advertise<sensor_msgs::PointCloud2>("m_local_map", 10);
  m_pointDriftPublisher = ph.advertise<sensor_msgs::PointCloud2>("point_drift_evaluation", 10);

  m_addKeyFrameService = ph.advertiseService("add_key_frame", &SlamNodelet::addKeyFrameServiceCall, this);
  m_addKeyFramesByDistanceService =
      ph.advertiseService("add_key_frames_by_distance", &SlamNodelet::addKeyFramesByDistanceServiceCall, this);
  m_clearSlamGraphService = ph.advertiseService("clear_graph", &SlamNodelet::clearSlamGraphServiceCall, this);
  m_resendKeyframesService = ph.advertiseService("resend_keyframes", &SlamNodelet::resendKeyframesServiceCall, this);
  m_evaluatePointDriftService =
      ph.advertiseService("evaluate_point_drift", &SlamNodelet::evaluatePointDriftServiceCall, this);

  ph.getParam("transform_wait_duration", transform_wait_duration_);
      
  // for registration
  ph.getParam("max_iter", m_maxIter);
  ph.getParam("prior_prob", m_priorProb);
  ph.getParam("soft_assoc_c1", m_softAssocC1);
  ph.getParam("soft_assoc_c2", m_softAssocC2);
  ph.getParam("sigma_size_factor", m_sigmaSizeFactor);

  // for adding and connecting edges
  ph.getParam("pose_is_close_dist", m_poseIsCloseDist);
  ph.getParam("pose_is_close_angle", m_poseIsCloseAngle);
  ph.getParam("pose_is_far_dist", m_poseIsFarDist);

  ph.param("height_image_frame", m_heightImageFrame, std::string("/map"));
  ph.param("height_image_size_x", m_heightImageSizeX, 8.0);
  ph.param("height_image_size_y", m_heightImageSizeY, 8.0);
  ph.param("height_image_resolution", m_heightImageResolution, 0.05);
  ph.param("height_image_min_z", m_heightImageMinZ, -1.0);
  ph.param("height_image_max_z", m_heightImageMaxZ, 0.0);
  ph.param("automatic_mode", m_addKeyFramesByDistance, true);

  initSLAMGraph();

  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_int_distribution<uint32_t> distribution;

  m_runID = distribution(mt);

  m_broadcasterThread =
      boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&SlamNodelet::broadcastTf, this)));
  m_processMapThread =
      boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&SlamNodelet::processMapQueue, this)));
}

void SlamNodelet::initSLAMGraph()
{
  m_firstMap = true;
  m_keyFrameCounter = 0;

  m_slam = boost::make_shared<mrsmap::SLAM>();

  m_slam->priorProb_ = m_priorProb;
  m_slam->softAssocC1_ = m_softAssocC1;
  m_slam->softAssocC2_ = static_cast<double>(m_softAssocC2);
  m_slam->sigmaSizeFactor_ = m_sigmaSizeFactor;

  m_slam->poseIsCloseDist_ = m_poseIsCloseDist;
  m_slam->poseIsCloseAngle_ = m_poseIsCloseAngle;
  m_slam->poseIsFarDist_ = m_poseIsFarDist;

  m_slam->addKeyFrameByDistance_ = m_addKeyFramesByDistance;

  m_worldCorrectedSlam.setIdentity();
}

void SlamNodelet::broadcastTf()
{
  ros::Rate loopRate(50);
  while (m_running)
  {
    {
      boost::unique_lock<boost::mutex> lock(m_correctionTransformMutex);
      // publish current slam pose
      tf_broadcaster_.sendTransform(
          tf::StampedTransform(m_worldCorrectedSlam, ros::Time::now(), m_odometryFrameId, m_slamMapFrameId));
      tf_listener_.setTransform(
          tf::StampedTransform(m_worldCorrectedSlam, ros::Time::now(), m_odometryFrameId, m_slamMapFrameId));
    }
    loopRate.sleep();
  }
}

bool SlamNodelet::addKeyFrameServiceCall(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  m_slam->addKeyFrame_ = true;
  return true;
}

bool SlamNodelet::addKeyFramesByDistanceServiceCall(mrs_laser_mapping::AddKeyFramesByDistance::Request& req,
                                                    mrs_laser_mapping::AddKeyFramesByDistance::Response& res)
{
  if (req.addKeyFrames == 0)
  {
    m_addKeyFramesByDistance = false;
  }
  else
  {
    m_addKeyFramesByDistance = true;
  }
  m_slam->addKeyFrameByDistance_ = m_addKeyFramesByDistance;
  return true;
}

bool SlamNodelet::clearSlamGraphServiceCall(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  initSLAMGraph();
  return true;
}

bool SlamNodelet::resendKeyframesServiceCall(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  m_keyFrameCounter = 0;
  return true;
}

bool SlamNodelet::evaluatePointDriftServiceCall(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  m_evaluatePointDrift = true;
  return true;
}

void SlamNodelet::receivedPoseUpdate(const geometry_msgs::PoseStamped& msg)
{
  NODELET_INFO("LocalizationNodelet::receivedPoseEstimate");

  Eigen::Matrix4d pose;

  pose(0, 3) = msg.pose.position.x;
  pose(1, 3) = msg.pose.position.y;
  pose(2, 3) = msg.pose.position.z;

  Eigen::Quaterniond q(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);

  pose.block<3, 3>(0, 0) = q.matrix();

  NODELET_INFO_STREAM("pose: " << pose << " header " << msg.header.frame_id);

  m_slam->setPose(pose);
}

void SlamNodelet::processMapQueue()
{
  while (m_running)
  {
    if (m_mapBuffer.size() > 3)
    {
      NODELET_ERROR_STREAM("Scans in scan buffer: " << m_mapBuffer.size());
    }

    MultiResolutionMapPtr map;
    m_mapBuffer.pop_back(&map);
    update(map);
  }
}

void SlamNodelet::receivedMap(const mrs_laser_mapping::MultiResolutionMapConstPtr& msg)
{
  NODELET_DEBUG_STREAM("received map from timestamp " << msg->header.stamp << " at time " << ros::Time::now());
  mrs_laser_mapping::MultiResolutionMapPtr map(new mrs_laser_mapping::MultiResolutionMap(*msg));
//   m_mapBuffer.push_front(map);
  update(map);
}

void SlamNodelet::update(const mrs_laser_mapping::MultiResolutionMapConstPtr& msg)
{
  ros::Time startTime = ros::Time::now();
  ros::Duration mapToNowDiff = ros::Time::now() - msg->header.stamp;

  if (mapToNowDiff.toSec() > 7.0)
  {
    NODELET_ERROR_STREAM("Skipping old map. Latency: " << mapToNowDiff);
    return;
  }
  else if (mapToNowDiff.toSec() > 4.0)
    NODELET_WARN_STREAM("Processing old map. Latency: " << mapToNowDiff);

  boost::shared_ptr<MapType> map = boost::shared_ptr<MapType>(
      new MapType(msg->size_in_meters, msg->resolution, msg->levels, msg->cell_capacity, msg->header.frame_id));

  for (unsigned int i = 0; i < msg->levels; i++)
  {
    pcl::PointCloud<mrs_laser_maps::MapPointType>::Ptr points(new pcl::PointCloud<mrs_laser_maps::MapPointType>());
    pcl::fromROSMsg(msg->cloud[i], *points);

    for (size_t j = 0; j < points->size(); j++)
    {
      map->setLevel(points->points[j], i);
    }
  }
  map->setLastUpdateTimestamp(msg->header.stamp);
  map->evaluateAll();
  NODELET_DEBUG_STREAM("created map from message: " << ros::Time::now() - startTime);

  // get current odometry transform
  tf::StampedTransform odomTransform;
  if (!getTranform(m_odometryFrameId, msg->header.frame_id, msg->header.stamp, odomTransform ))
    return;

  Eigen::Affine3d odomTransformEigen;
  tf::transformTFToEigen(odomTransform, odomTransformEigen);

  // get current base_link_oriented transform from the slam frame at last scans time
  tf::StampedTransform slamFrameBloTransform;
  if (!getTranform(m_slamMapFrameId, msg->header.frame_id, msg->header.stamp, slamFrameBloTransform ))
    return;

  Eigen::Affine3d slamFrameBloEigen;
  tf::transformTFToEigen(slamFrameBloTransform, slamFrameBloEigen);

  // use estimated transform from local mapping as initial transform guess
  tf::StampedTransform baseLinkOrientedTransform;
  if (!getTranform("/world_corrected", msg->header.frame_id, msg->header.stamp, baseLinkOrientedTransform))
    return;

  Eigen::Affine3d stampedTransformOrientedEigen;
  tf::transformTFToEigen(baseLinkOrientedTransform, stampedTransformOrientedEigen);

  Eigen::Affine3d lastScanTransformEigen;
  tf::transformTFToEigen(m_lastBaseLinkOrientedTransform, lastScanTransformEigen);

  Eigen::Matrix4d baseLinkOrientedDeltaTransform =
      lastScanTransformEigen.matrix().inverse() * stampedTransformOrientedEigen.matrix();

  if (m_firstMap)
  {
    m_firstMap = false;
    m_startTime = msg->header.stamp;
  }
  else
  {
    // update tracked pose by registration result from the local mapping
    g2o::VertexSE3* v_ref = dynamic_cast<g2o::VertexSE3*>(
        m_slam->optimizer_->vertex(m_slam->keyFrames_[m_slam->referenceKeyFrameId_]->nodeId_));
    Eigen::Matrix4d referenceKeyFrameTransform = v_ref->estimate().matrix();
    Eigen::Matrix4d referenceInBaseLinkOriented = slamFrameBloEigen.inverse() * referenceKeyFrameTransform;
    m_slam->lastTransform_ = referenceInBaseLinkOriented.inverse() * baseLinkOrientedDeltaTransform *
                             referenceInBaseLinkOriented * m_slam->lastTransform_;

    g2o::VertexSE3* vReference = dynamic_cast<g2o::VertexSE3*>(
        m_slam->optimizer_->vertex(m_slam->keyFrames_[m_slam->referenceKeyFrameId_]->nodeId_));

    // publish the reference keyframe
    pcl::PointCloud<mrs_laser_maps::MapPointType>::Ptr cellPointsCloudReference(new pcl::PointCloud<mrs_laser_maps::MapPointType>());
    cellPointsCloudReference->header.stamp = pcl_conversions::toPCL(m_lastUpdateTime);
    cellPointsCloudReference->header.frame_id = m_slamMapFrameId;  // slam->keyFrames_[ 0 ]->map_->getFrameId();
    m_slam->keyFrames_[m_slam->referenceKeyFrameId_]->map_->getCellPointsDownsampled(cellPointsCloudReference, 100);
    pcl::transformPointCloud(*cellPointsCloudReference, *cellPointsCloudReference,
                             vReference->estimate().matrix().cast<float>());
    m_referenceKeyframePublisher.publish(cellPointsCloudReference);
  }

  m_lastBaseLinkOrientedTransform = baseLinkOrientedTransform;

  NODELET_DEBUG_STREAM("received map before update: " << ros::Time::now() - startTime);

  pcl::PointCloud<mrs_laser_maps::MapPointType>::Ptr cloud(new pcl::PointCloud<mrs_laser_maps::MapPointType>());
  // update the slam graph and track the pose
  m_slam->update(map, cloud);

  NODELET_DEBUG_STREAM("received map after update: " << ros::Time::now() - startTime);

//   if ((m_mapBuffer.size() < 1))
//     m_slam->refine(3, 20, 100, 100);
  
  
//   Eigen::Matrix4d sensorTransformEigen;
//   // if (!m_hasSensorTransform)
//   {
//     try
//     {
//       //
//       Eigen::Affine3d sensorTransformEigenAffine;
//       tf::StampedTransform sensorTransform;
//       if (m_tfListener.waitForTransform("laser_scanner_origin", "/base_link_oriented", ros::Time(0),
//                                         ros::Duration(0.2)))
//       {
//         m_tfListener.lookupTransform("laser_scanner_origin", "/base_link_oriented", ros::Time(0), sensorTransform);
//         tf::transformTFToEigen(sensorTransform, sensorTransformEigenAffine);
//         sensorTransformEigen = sensorTransformEigenAffine.matrix();
//         // 					m_hasSensorTransform = true;
//       }
//       else
//       {
//         NODELET_ERROR("registerScans:: could not wait for sensor transform");
//         return;
//       }
//     }
//     catch (tf::TransformException ex)
//     {
//       NODELET_ERROR("registerScans:: could not lookup sensor transform: %s", ex.what());
//       return;
//     }
//   }


  NODELET_DEBUG_STREAM("publishing slam map from timestamp " << msg->header.stamp << " at time " << ros::Time::now());
  publishGraph();
  //  		publishSLAMGraph();

  m_lastUpdateTime = msg->header.stamp;

  // get current slam pose to broadcast /world_corrected_slam
  g2o::VertexSE3* v_curr = dynamic_cast<g2o::VertexSE3*>(
      m_slam->optimizer_->vertex(m_slam->keyFrames_[m_slam->referenceKeyFrameId_]->nodeId_));
  Eigen::Matrix4d camTransform = v_curr->estimate().matrix();
  Eigen::Matrix4d lasTransform = camTransform * m_slam->lastTransform_;

  {
    // update m_worldCorrectedSlam by current slam pose and odometry (since published in odom frame)
    boost::unique_lock<boost::mutex> lock(m_correctionTransformMutex);
    tf::transformEigenToTF(Eigen::Affine3d((lasTransform * odomTransformEigen.matrix().inverse())).inverse(),
                           m_worldCorrectedSlam);
  }
  NODELET_DEBUG_STREAM("finished slam procesing from timestamp " << msg->header.stamp << " at time "
                                                                 << ros::Time::now());

  // publish odometry message
  tf::StampedTransform baseLinkInWorldTransform;
  {
    if (tf_listener_.waitForTransform(m_slamMapFrameId, "/base_link", msg->header.stamp, ros::Duration(0.5f)))
    {
      tf_listener_.lookupTransform(m_slamMapFrameId, "/base_link", msg->header.stamp, baseLinkInWorldTransform);
    }
    else
    {
      NODELET_ERROR_STREAM("slam_nodelet: base_link in in world: error in getting transform");
    }
  }

  nav_msgs::Odometry odomMsg;
  odomMsg.header.stamp = msg->header.stamp;
  odomMsg.header.frame_id = m_slamMapFrameId;

  tf::poseTFToMsg(baseLinkInWorldTransform, odomMsg.pose.pose);
  m_odometryPublisher.publish(odomMsg);

  m_odometryMsgs.push_back(odomMsg);
}

void SlamNodelet::publishOdometryGraph()
{
  visualization_msgs::Marker points, line_list, cylinders, innerCylinders;
  points.header.frame_id = line_list.header.frame_id = cylinders.header.frame_id = innerCylinders.header.frame_id =
      m_slamMapFrameId;  // slam->keyFrames_[ 0 ]->map_->getFrameId(); // "/slam_frame"; //

  points.header.stamp = line_list.header.stamp = cylinders.header.stamp = innerCylinders.header.stamp =
      m_slam->keyFrames_[0]->map_->getLastUpdateTimestamp();
  points.ns = line_list.ns = cylinders.ns = innerCylinders.ns = "localization_marker";
  points.action = line_list.action = cylinders.action = innerCylinders.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_list.pose.orientation.w = cylinders.pose.orientation.w =
      innerCylinders.pose.orientation.w = 1.0;

  unsigned int markerId = 1;
  const double zOverlayOffset = 0.01;
  double zOverlay = 0.0;

  line_list.id = markerId++;

  points.type = visualization_msgs::Marker::LINE_LIST;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  cylinders.type = visualization_msgs::Marker::CYLINDER;
  innerCylinders.type = visualization_msgs::Marker::CYLINDER;

  points.scale.x = 0.025;

  points.color.r = 0.f;
  points.color.g = 0.f;
  points.color.b = 0.f;
  points.color.a = 1.0f;

  line_list.scale.x = 0.05;

  line_list.color.r = 0.5f;
  line_list.color.g = 0.5f;
  line_list.color.b = 0.5f;
  line_list.color.a = 1.0f;

  cylinders.scale.x = 0.25;
  cylinders.scale.y = 0.25;
  cylinders.scale.z = 0.005;

  cylinders.color.r = 0.f;
  cylinders.color.g = 0.f;
  cylinders.color.b = 0.f;
  cylinders.color.a = 1.0f;

  innerCylinders.scale.x = 0.2;
  innerCylinders.scale.y = 0.2;
  innerCylinders.scale.z = 0.005;

  innerCylinders.color.r = 1.f;
  innerCylinders.color.g = 1.f;
  innerCylinders.color.b = 1.f;
  innerCylinders.color.a = 1.0f;

  geometry_msgs::Point lastPoint;
  for (unsigned int i = 1; i < m_odometryMsgs.size(); i++)
  {
    geometry_msgs::Point p = m_odometryMsgs[i].pose.pose.position;

    Eigen::Vector3d lastPointEigen(lastPoint.x, lastPoint.y, lastPoint.z);
    Eigen::Vector3d pointEigen(p.x, p.y, p.z);

    if ((i > 1) && (pointEigen - lastPointEigen).norm() < cylinders.scale.x)
      continue;

    zOverlay = 0.0;
    cylinders.pose = m_odometryMsgs[i].pose.pose;
    zOverlay += zOverlayOffset;
    cylinders.pose.position.z += zOverlay;
    cylinders.id = markerId++;
    m_slamGraphMarkerPublisher.publish(cylinders);

    innerCylinders.pose = m_odometryMsgs[i].pose.pose;

    zOverlay += zOverlayOffset;
    innerCylinders.pose.position.z += zOverlay;
    innerCylinders.id = markerId++;

    // time delta
    ros::Duration duration = m_odometryMsgs[i].header.stamp - m_startTime;

    float r, g, b;
    getRainbowColor((duration.toSec() / (60 * 20)), r, g, b);

    innerCylinders.color.r = r;
    innerCylinders.color.g = g;
    innerCylinders.color.b = b;
    innerCylinders.color.a = 1.0f;

    m_slamGraphMarkerPublisher.publish(innerCylinders);

    /*
    geometry_msgs::Pose p2Pose = m_odometryMsgs[i].pose.pose;
    Eigen::Affine3d p2Eigen;
    tf::poseMsgToEigen( p2Pose, p2Eigen);
    */
    Eigen::Matrix4d p2EigenMatrix = Eigen::Matrix4d::Identity();  // p2Eigen.matrix();

    Eigen::Quaterniond q(m_odometryMsgs[i].pose.pose.orientation.w, m_odometryMsgs[i].pose.pose.orientation.x,
                         m_odometryMsgs[i].pose.pose.orientation.y, m_odometryMsgs[i].pose.pose.orientation.z);

    p2EigenMatrix.block<3, 3>(0, 0) = q.matrix();

    // 			Eigen::Vector4d pEigen(cylinders.scale.x/4, 0.0, 0.0, 1.0);
    //  			pEigen = p2EigenMatrix * pEigen ;
    // 			p.x += pEigen(0);
    // 			p.y += pEigen(1);
    // 			p.z += pEigen(2);
    //

    Eigen::Vector4d p2Eigen(cylinders.scale.x * 0.5, 0.0, 0.0, 1.0);
    p2Eigen = p2EigenMatrix * p2Eigen;

    geometry_msgs::Point p2 = m_odometryMsgs[i].pose.pose.position;
    p2.x += p2Eigen(0);
    p2.y += p2Eigen(1);
    p2.z += p2Eigen(2);

    if (i >= 2)
    {
      line_list.points.push_back(lastPoint);
      line_list.points.push_back(p);
    }

    lastPoint = p;

    // add something to z-component to overlay cylinders
    zOverlay += zOverlayOffset;
    p.z += zOverlay;
    p2.z += zOverlay;
    points.points.push_back(p);
    points.points.push_back(p2);
  }

  points.id = markerId++;
  m_slamGraphMarkerPublisher.publish(points);
  m_slamGraphMarkerPublisher.publish(line_list);
}

void SlamNodelet::publishSLAMGraph()
{
  pcl::PointCloud<mrs_laser_maps::MapPointType>::Ptr cloud_final =
      pcl::PointCloud<mrs_laser_maps::MapPointType>::Ptr(new pcl::PointCloud<mrs_laser_maps::MapPointType>());
  pcl::PointCloud<mrs_laser_maps::MapPointType>::Ptr cloudFinalDownsampled =
      pcl::PointCloud<mrs_laser_maps::MapPointType>::Ptr(new pcl::PointCloud<mrs_laser_maps::MapPointType>());

  visualization_msgs::Marker points, line_list;
  points.header.frame_id = line_list.header.frame_id =
      m_slamMapFrameId;  // slam->keyFrames_[ 0 ]->map_->getFrameId(); // "/slam_frame"; //

  points.header.stamp = line_list.header.stamp = m_slam->keyFrames_[0]->map_->getLastUpdateTimestamp();
  points.ns = line_list.ns = "points_and_lines";
  points.action = line_list.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_list.pose.orientation.w = 1.0;

  points.id = 0;
  line_list.id = 1;

  points.type = visualization_msgs::Marker::POINTS;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  points.scale.x = 0.1;
  points.scale.y = 0.1;

  line_list.scale.x = 0.05;

  points.color.r = 1.0f;
  points.color.g = 1.f;
  points.color.b = 0.f;
  points.color.a = 1.f;

  line_list.color.r = 0.0f;
  line_list.color.g = 0.0f;
  line_list.color.b = 0.0f;
  line_list.color.a = 1.0f;

  pcl::PointCloud<mrs_laser_maps::MapPointType>::Ptr referenceKeyframeCloud =
      pcl::PointCloud<mrs_laser_maps::MapPointType>::Ptr(new pcl::PointCloud<mrs_laser_maps::MapPointType>());

  // 		mrs_laser_maps::MultiResolutionalMap mapFull(64, 128, 1, 1000, m_slamMapFrameId);
  // 		Eigen::Matrix4d sensorTransformEigen;
  // 		//if (!m_hasSensorTransform)
  // 		{
  // 			try
  // 			{
  // //
  // 				Eigen::Affine3d sensorTransformEigenAffine;
  // 				tf::StampedTransform sensorTransform;
  // 				if ( m_tfListener.waitForTransform( "laser_scanner_origin", "/base_link_oriented", ros::Time(0) ,
  // ros::Duration( 0.2 ) ) ) {
  // 					m_tfListener.lookupTransform( "laser_scanner_origin", "/base_link_oriented", ros::Time(0), sensorTransform
  // );
  // 					tf::transformTFToEigen( sensorTransform, sensorTransformEigenAffine );
  // 					sensorTransformEigen = sensorTransformEigenAffine.matrix();
  // // 					m_hasSensorTransform = true;
  // 				}
  // 				else {
  // 					NODELET_ERROR("registerScans:: could not wait for sensor transform");
  // 					return;
  // 				}
  // 			}
  // 			catch (tf::TransformException ex)
  // 			{
  // 				NODELET_ERROR("registerScans:: could not lookup sensor transform: %s", ex.what());
  // 				return;
  // 			}
  // 		}

  for (unsigned int i = 0; i < m_slam->keyFrames_.size(); i++)
  {
    g2o::VertexSE3* v_curr = dynamic_cast<g2o::VertexSE3*>(m_slam->optimizer_->vertex(m_slam->keyFrames_[i]->nodeId_));

    // add coordinate frames for vertices
    Eigen::Matrix4d camTransform = v_curr->estimate().matrix();

    geometry_msgs::Point p;
    p.x = camTransform(0, 3);
    p.y = camTransform(1, 3);
    p.z = camTransform(2, 3);

    points.points.push_back(p);

    if (i == m_slam->referenceKeyFrameId_)
    {
      pcl::transformPointCloud(*m_slam->keyFrames_[i]->cloud_, *referenceKeyframeCloud, camTransform.cast<float>());
    }

    // publish downsampled map
    pcl::PointCloud<mrs_laser_maps::MapPointType>::Ptr cellPointsCloudDownsampled(new pcl::PointCloud<mrs_laser_maps::MapPointType>());
    m_slam->keyFrames_[i]->map_->getCellPointsDownsampled(cellPointsCloudDownsampled, 100);
    pcl::PointCloud<mrs_laser_maps::MapPointType>::Ptr transformedCloudDownsampled =
        pcl::PointCloud<mrs_laser_maps::MapPointType>::Ptr(new pcl::PointCloud<mrs_laser_maps::MapPointType>());
    pcl::transformPointCloud(*cellPointsCloudDownsampled, *transformedCloudDownsampled, camTransform.cast<float>());

    // 			mapFull.setOccupancyParameters( -2.f, 3.5, 0.85, -0.4f);
    //
    // 			mapFull.addCloud( transformedCloudDownsampled, true, camTransform * sensorTransformEigen.inverse());
    //

    *cloudFinalDownsampled += *transformedCloudDownsampled;
    //

    // 			mapDownsampled.addCloud( cellPointsCloud );
    // 			pcl::PointCloud< pcl::PointXYZ >::Ptr cellPointsCloudDownsampled(new pcl::PointCloud< pcl::PointXYZ >() );
    // 			mapDownsampled.getCellPoints( cellPointsCloudDownsampled );
    //
    // 			pcl::PointCloud< pcl::PointXYZ >::Ptr transformedCloud = pcl::PointCloud< pcl::PointXYZ >::Ptr( new
    // pcl::PointCloud< pcl::PointXYZ >() );
    // 			pcl::transformPointCloud( *cellPointsCloudDownsampled, *transformedCloud, camTransform.cast< float >() );

    pcl::PointCloud<mrs_laser_maps::MapPointType>::Ptr cellPointsCloud(new pcl::PointCloud<mrs_laser_maps::MapPointType>());
    m_slam->keyFrames_[i]->map_->getCellPoints(cellPointsCloud);
    pcl::PointCloud<mrs_laser_maps::MapPointType>::Ptr transformedCloud =
        pcl::PointCloud<mrs_laser_maps::MapPointType>::Ptr(new pcl::PointCloud<mrs_laser_maps::MapPointType>());
    pcl::transformPointCloud(*cellPointsCloud, *transformedCloud, camTransform.cast<float>());

    *cloud_final += *transformedCloud;
  }

  // 		cloudFinalDownsampled->clear();
  // 		mapFull.getCellPoints( cloudFinalDownsampled );

  /*
   * graph edges
   */
  for (EdgeSet::iterator it = m_slam->optimizer_->edges().begin(); it != m_slam->optimizer_->edges().end(); ++it)
  {
    g2o::EdgeSE3* edge = dynamic_cast<g2o::EdgeSE3*>(*it);

    // add lines for edges
    g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge->vertices()[0]);
    g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(edge->vertices()[1]);

    // add coordinate frames for vertices
    Eigen::Matrix4d pose1 = v1->estimate().matrix();
    Eigen::Matrix4d pose2 = v2->estimate().matrix();

    geometry_msgs::Point p1, p2;
    p1.x = pose1(0, 3);
    p1.y = pose1(1, 3);
    p1.z = pose1(2, 3);
    p2.x = pose2(0, 3);
    p2.y = pose2(1, 3);
    p2.z = pose2(2, 3);

    line_list.points.push_back(p1);
    line_list.points.push_back(p2);
  }

  m_slamGraphMarkerPublisher.publish(points);
  m_slamGraphMarkerPublisher.publish(line_list);

  cloud_final->header.stamp = pcl_conversions::toPCL(m_slam->keyFrames_[0]->map_->getLastUpdateTimestamp());
  cloud_final->header.frame_id = m_slamMapFrameId;  // slam->keyFrames_[ 0 ]->map_->getFrameId();
  m_mapPublisher.publish(cloud_final);

  // 		pcl::PointCloud< pcl::PointXYZ >::Ptr transformed(new pcl::PointCloud< pcl::PointXYZ >());
  // 		transformed->header.stamp = pcl_conversions::toPCL( m_slam->keyFrames_[ 0 ]->map_->getLastUpdateTimestamp() );
  //  		transformed->header.frame_id = "world_corrected";//slam->keyFrames_[ 0 ]->map_->getFrameId();
  //
  // 		if(!m_tfListener.waitForTransform("world_corrected", "map",  m_slam->keyFrames_[ 0
  // ]->map_->getLastUpdateTimestamp() , ros::Duration(4.0)))
  // 		{
  // 			NODELET_ERROR("Could not wait for transform");
  // 			return;
  // 		}
  // 		pcl_ros::transformPointCloud("world_corrected", *cloud_final, *transformed, m_tfListener);
  //
  //  		m_mapPublisher.publish(transformed);

  cloudFinalDownsampled->header.stamp = pcl_conversions::toPCL(m_slam->keyFrames_[0]->map_->getLastUpdateTimestamp());
  cloudFinalDownsampled->header.frame_id = m_slamMapFrameId;  // slam->keyFrames_[ 0 ]->map_->getFrameId();
  m_mapDownsampledPublisher.publish(cloudFinalDownsampled);

  referenceKeyframeCloud->header.stamp = pcl_conversions::toPCL(m_slam->keyFrames_[0]->map_->getLastUpdateTimestamp());
  referenceKeyframeCloud->header.frame_id = m_slamMapFrameId;  // slam->keyFrames_[ 0 ]->map_->getFrameId();
  m_referenceKeyframePublisher.publish(referenceKeyframeCloud);

  // 		buildHeigImage( cloud_final );
}

void SlamNodelet::publishGraph()
{
  /*
   * publish every keyframe that has not been published yet
   */
  for (unsigned int i = m_keyFrameCounter; i < m_slam->keyFrames_.size(); i++)
  {
    pcl::PointCloud<mrs_laser_maps::MapPointType>::Ptr cellPointCloud(new pcl::PointCloud<mrs_laser_maps::MapPointType>());
    m_slam->keyFrames_[i]->map_->getCellPoints(cellPointCloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cellPointCloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*cellPointCloud, *cellPointCloudRGB);

    mrs_laser_mapping::KeyFrame keyFrameMsg;
    keyFrameMsg.header.stamp = m_lastUpdateTime;
    keyFrameMsg.header.frame_id = m_slamMapFrameId;
    keyFrameMsg.runID = m_runID;
    keyFrameMsg.keyframeID = m_keyFrameCounter++;

    Compression compression;
    VectorStream stream(&keyFrameMsg.compressedCloud);
    compression.encodePointCloud(cellPointCloudRGB, stream);

    m_keyFramePublisher.publish(keyFrameMsg);

//     // if a new keyframe was added, let the thread calculate the point drift
//     m_evaluatePointDrift = true;
  }

  /*
   * publish transforms for each key frame and graph edges on each update (change for every optimizer run)
   */
  mrs_laser_mapping::KeyFrameTransforms transformsMsg;
  transformsMsg.header.stamp = m_lastUpdateTime;
  transformsMsg.header.frame_id = m_slamMapFrameId;

  // key frame transforms
  for (unsigned int i = 0; i < m_slam->keyFrames_.size(); i++)
  {
    g2o::VertexSE3* v_curr = dynamic_cast<g2o::VertexSE3*>(m_slam->optimizer_->vertex(m_slam->keyFrames_[i]->nodeId_));

    Eigen::Matrix4d nodeTransform = v_curr->estimate().matrix();

    geometry_msgs::Transform nodeTransformMsg;
    tf::transformEigenToMsg(Eigen::Affine3d(nodeTransform), nodeTransformMsg);
    transformsMsg.transforms.push_back(nodeTransformMsg);
  }

  // graph edges
  for (EdgeSet::iterator it = m_slam->optimizer_->edges().begin(); it != m_slam->optimizer_->edges().end(); ++it)
  {
    g2o::EdgeSE3* edge = dynamic_cast<g2o::EdgeSE3*>(*it);
    g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge->vertices()[0]);
    g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(edge->vertices()[1]);

    // add coordinate frames for vertices
    Eigen::Matrix4d pose1 = v1->estimate().matrix();
    Eigen::Matrix4d pose2 = v2->estimate().matrix();

    geometry_msgs::Point p1, p2;
    p1.x = pose1(0, 3);
    p1.y = pose1(1, 3);
    p1.z = pose1(2, 3);
    p2.x = pose2(0, 3);
    p2.y = pose2(1, 3);
    p2.z = pose2(2, 3);

    transformsMsg.edgeStartPoints.push_back(p1);
    transformsMsg.edgeEndPoints.push_back(p2);
  }

  m_keyFrameTransformsPublisher.publish(transformsMsg);
}


bool SlamNodelet::getTranform(const std::string& target_frame, const std::string& source_frame, ros::Time time, Eigen::Matrix4f& transform)
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
