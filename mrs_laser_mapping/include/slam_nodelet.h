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

#include <ros/ros.h>

#include <mrs_laser_maps/multiresolution_surfel_registration.h>

#include <mrs_laser_maps/map_multiresolution.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <mrs_laser_maps/synchronized_circular_buffer.h>

#include <message_filters/subscriber.h>

#include <tf/tfMessage.h>
#include <tf/message_filter.h>
#include <tf_conversions/tf_eigen.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <nodelet/nodelet.h>

#include <std_srvs/Empty.h>
#include <mrs_laser_mapping/MultiResolutionMap.h>
#include <mrs_laser_mapping/KeyFrame.h>
#include <mrs_laser_mapping/KeyFrameTransforms.h>
#include <mrs_laser_mapping/AddKeyFramesByDistance.h>

#include <config_server/parameter.h>

#include "mrsmap/slam.h"

#include <cloud_compression/octree_pointcloud_compression.h>

namespace mrs_laser_mapping
{
class SlamNodelet : public nodelet::Nodelet
{
public:
  typedef mrs_laser_maps::MapPointType MapPointType;
  typedef mrs_laser_maps::InputPointType InputPointType;
  typedef mrs_laser_maps::MapType MapType;
  typedef MapType::GridCellType GridCellType;

  SlamNodelet();
  virtual ~SlamNodelet();
  virtual void onInit();

  void initSLAMGraph();

  bool addKeyFrameServiceCall(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool addKeyFramesByDistanceServiceCall(mrs_laser_mapping::AddKeyFramesByDistance::Request& req,
                                         mrs_laser_mapping::AddKeyFramesByDistance::Response& res);
  bool clearSlamGraphServiceCall(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool resendKeyframesServiceCall(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  bool evaluatePointDriftServiceCall(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  void processMapQueue();
  void receivedMap(const mrs_laser_mapping::MultiResolutionMapConstPtr& msg);
  void update(const mrs_laser_mapping::MultiResolutionMapConstPtr& msg);

  void receivedPoseUpdate(const geometry_msgs::PoseStamped& msg);

  void publishSLAMGraph();
  void publishGraph();
  void publishOdometryGraph();

  void broadcastTf();

  bool getTranform(const std::string& target_frame, const std::string& source_frame, ros::Time time, Eigen::Matrix4f& transform);
  bool getTranform(const std::string& target_frame, const std::string& source_frame, ros::Time time, tf::StampedTransform& transform);
  
//   void evaluateInterMapPointDrift();

protected:
  static void getRainbowColor(float value, float& r, float& g, float& b)
  {
    // this is HSV color palette with hue values going only from 0.0 to 0.833333.
    value = std::min(value, 1.0f);
    value = std::max(value, 0.0f);

    float color[3];

    float h = value * 5.0f + 1.0f;
    int i = floor(h);
    float f = h - i;
    if (!(i & 1))
      f = 1 - f;  // if i is even
    float n = 1 - f;

    if (i <= 1)
      color[0] = n, color[1] = 0, color[2] = 1;
    else if (i == 2)
      color[0] = 0, color[1] = n, color[2] = 1;
    else if (i == 3)
      color[0] = 0, color[1] = 1, color[2] = n;
    else if (i == 4)
      color[0] = n, color[1] = 1, color[2] = 0;
    else if (i >= 5)
      color[0] = 1, color[1] = n, color[2] = 0;

    r = color[0];
    g = color[1];
    b = color[2];
  }

private:
  volatile bool m_running;

  boost::shared_ptr<boost::thread> m_broadcasterThread;
  boost::mutex m_correctionTransformMutex;

  ros::Subscriber m_mapSubscriber;
  ros::Subscriber m_poseUpdateSubscriber;

  ros::Publisher m_slamGraphMarkerPublisher;
  ros::Publisher m_mapPublisher;
  ros::Publisher m_mapDownsampledPublisher;
  ros::Publisher m_referenceKeyframePublisher;
  ros::Publisher m_heightImagePublisher;
  ros::Publisher m_keyFramePublisher;
  ros::Publisher m_keyFrameTransformsPublisher;
  ros::Publisher m_odometryPublisher;
  ros::Publisher m_localMapPublisher;
  ros::Publisher m_pointDriftPublisher;

  ros::ServiceServer m_clearSlamGraphService;
  ros::ServiceServer m_addKeyFrameService;
  ros::ServiceServer m_addKeyFramesByDistanceService;
  ros::ServiceServer m_resendKeyframesService;
  ros::ServiceServer m_evaluatePointDriftService;

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;

  double transform_wait_duration_;
  
  tf::StampedTransform m_lastBaseLinkOrientedTransform;
  tf::StampedTransform m_worldCorrectedSlam;

  std::string m_slamMapFrameId;
  std::string m_heightImageFrame;
  std::string m_odometryFrameId;

  boost::shared_ptr<mrsmap::SLAM> m_slam;

  int m_maxIter;
  double m_priorProb;
  double m_softAssocC1;
  int m_softAssocC2;
  double m_sigmaSizeFactor;

  double m_poseIsCloseDist;
  double m_poseIsCloseAngle;

  double m_poseIsFarDist;

  double m_heightImageSizeX;
  double m_heightImageSizeY;
  double m_heightImageResolution;
  double m_heightImageMinZ;
  double m_heightImageMaxZ;

  bool m_firstMap;
  bool m_addKeyFramesByDistance;

  uint32_t m_runID;
  uint32_t m_keyFrameCounter;

  typedef cloud_compression::OctreePointCloudCompression<pcl::PointXYZRGB> Compression;

  std::vector<nav_msgs::Odometry> m_odometryMsgs;

  ros::Time m_lastUpdateTime;
  ros::Time m_startTime;

  mrs_laser_maps::synchronized_circular_buffer<mrs_laser_mapping::MultiResolutionMapPtr> m_mapBuffer;
  boost::mutex m_mapBufferMutex;
  boost::shared_ptr<boost::thread> m_processMapThread;

  bool m_evaluatePointDrift;
};
}

#endif
