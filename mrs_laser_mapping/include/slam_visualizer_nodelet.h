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

#ifndef _SLAM_VISUALIZER_NODELET_H_
#define _SLAM_VISUALIZER_NODELET_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_laser_maps/map_multiresolution.h>

#include <mrs_laser_maps/synchronized_circular_buffer.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <message_filters/subscriber.h>

#include <tf/tfMessage.h>
#include <tf/message_filter.h>
#include <tf_conversions/tf_eigen.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud.h>

#include <visualization_msgs/Marker.h>

#include <std_srvs/Empty.h>
#include <mrs_laser_mapping/MultiResolutionMap.h>
#include <mrs_laser_mapping/KeyFrame.h>
#include <mrs_laser_mapping/KeyFrameTransforms.h>

#include <config_server/parameter.h>

#include <cloud_compression/octree_pointcloud_compression.h>

namespace mrs_laser_mapping
{
class SlamVisualizerNodelet : public nodelet::Nodelet
{
public:
  SlamVisualizerNodelet();
  virtual ~SlamVisualizerNodelet();
  virtual void onInit();

  void receivedKeyFrame(const mrs_laser_mapping::KeyFrameConstPtr& msg);
  void receivedKeyFrameTransforms(const mrs_laser_mapping::KeyFrameTransformsConstPtr& msg);

  void processKeyframeQueue();
  void processKeyframeTransformQueue();

  void processKeyFrame(const mrs_laser_mapping::KeyFrameConstPtr& msg);
  void processKeyFrameTransforms(const mrs_laser_mapping::KeyFrameTransformsConstPtr& msg);

protected:
  void HSVtoRGB(float h, float s, float v, float* r, float* g, float* b)
  {
    int i;
    float f, p, q, t;
    if (s == 0)
    {
      // achromatic (grey)
      *r = *g = *b = v;
      return;
    }
    h /= 60;  // sector 0 to 5
    i = floor(h);
    f = h - i;  // factorial part of h
    p = v * (1 - s);
    q = v * (1 - s * f);
    t = v * (1 - s * (1 - f));
    switch (i)
    {
      case 0:
        *r = v;
        *g = t;
        *b = p;
        break;
      case 1:
        *r = q;
        *g = v;
        *b = p;
        break;
      case 2:
        *r = p;
        *g = v;
        *b = t;
        break;
      case 3:
        *r = p;
        *g = q;
        *b = v;
        break;
      case 4:
        *r = t;
        *g = p;
        *b = v;
        break;
      default:  // case 5:
        *r = v;
        *g = p;
        *b = q;
        break;
    }
  }

private:
  typedef pcl::PointXYZRGB PointType;
  typedef pcl::PointCloud<PointType> PointCloud;
  typedef PointCloud::Ptr PointCloudPtr;

  volatile bool m_running;

  ros::Subscriber m_keyFrameSubscriber;
  ros::Subscriber m_keyFrameTransformsSubscriber;

  ros::Publisher m_slamGraphMarkerPublisher;
  ros::Publisher m_mapPublisher;
  ros::Publisher m_mapDownsampledPublisher;
  ros::Publisher m_mapColorShadingPublisher;

  tf::TransformListener m_tfListener;

  std::string m_slamMapFrameId;

  double m_voxelLeafSize;

  float m_filterLimitMinX;
  float m_filterLimitMaxX;
  float m_filterLimitMinY;
  float m_filterLimitMaxY;
  float m_filterLimitMinZ;
  float m_filterLimitMaxZ;

  uint32_t m_runID;
  uint32_t m_keyFrameCounter;

  typedef cloud_compression::OctreePointCloudCompression<PointType> Compression;
  Compression::Ptr m_compression;

  std::vector<PointCloudPtr> m_keyFrameClouds;
  std::vector<PointCloudPtr> m_keyFrameCloudsDownsampled;

  std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> m_keyFrameCloudsDownsampledNormals;

  float m_minZ;
  float m_maxZ;

  std::vector<Eigen::Matrix4d> m_keyFrameTransforms;

  mrs_laser_maps::synchronized_circular_buffer<mrs_laser_mapping::KeyFramePtr> m_keyframeBuffer;
  boost::shared_ptr<boost::thread> m_processKeyframeThread;

  mrs_laser_maps::synchronized_circular_buffer<mrs_laser_mapping::KeyFrameTransformsPtr> m_keyframeTransformBuffer;
  boost::shared_ptr<boost::thread> m_processKeyframeTransformThread;

  boost::mutex m_keyframeMutex;
};
}

#endif
