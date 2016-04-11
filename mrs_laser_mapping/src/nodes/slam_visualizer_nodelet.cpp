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

#include <slam_visualizer_nodelet.h>
// #include <voxel_grid_statistics.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>

#include <eigen_conversions/eigen_msg.h>

#include <pluginlib/class_list_macros.h>

typedef pcl::PointXYZ PointT;

namespace
{
class VectorStreamBuf : public std::streambuf
{
public:
  explicit VectorStreamBuf(const std::vector<uint8_t>* in)
  {
    setg(reinterpret_cast<char*>(const_cast<uint8_t*>(in->data())),
         reinterpret_cast<char*>(const_cast<uint8_t*>(in->data())),
         reinterpret_cast<char*>(const_cast<uint8_t*>(in->data() + in->size())));
  }
};

class VectorStream : public std::istream
{
public:
  explicit VectorStream(const std::vector<uint8_t>* in) : m_buf(in)
  {
    rdbuf(&m_buf);
  }

private:
  VectorStreamBuf m_buf;
};
}

namespace mrs_laser_mapping
{
SlamVisualizerNodelet::SlamVisualizerNodelet()
  : m_running(true)
  , m_slamMapFrameId("/world_corrected_slam")
  , m_runID(0)
  , m_keyFrameCounter(0)
  , m_filterLimitMinX(-8.f)
  , m_filterLimitMaxX(8.f)
  , m_filterLimitMinY(-8.f)
  , m_filterLimitMaxY(8.f)
  , m_filterLimitMinZ(-8.f)
  , m_filterLimitMaxZ(8.f)
	, m_minZ(std::numeric_limits<float>::max())
  , m_maxZ(std::numeric_limits<float>::min())
	, m_keyframeBuffer(10)
  , m_keyframeTransformBuffer(10)
{
  NODELET_INFO("Initializing nodelet.. ");

  m_compression.reset(new Compression(pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR));
}

SlamVisualizerNodelet::~SlamVisualizerNodelet()
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

void SlamVisualizerNodelet::onInit()
{
  NODELET_INFO("SlamVisualizerNodelet::onInit nodelet...");

  ros::NodeHandle& ph = getMTPrivateNodeHandle();

  ph.getParam("slam_frame_id", m_slamMapFrameId);

  m_keyFrameSubscriber = ph.subscribe("keyframes", 1, &SlamVisualizerNodelet::receivedKeyFrame, this);
  m_keyFrameTransformsSubscriber =
      ph.subscribe("keyframe_transforms", 1, &SlamVisualizerNodelet::receivedKeyFrameTransforms, this);

  m_slamGraphMarkerPublisher = ph.advertise<visualization_msgs::Marker>("slam_graph_marker", 10);
  m_mapPublisher = ph.advertise<sensor_msgs::PointCloud2>("slam_map", 10);
  m_mapDownsampledPublisher = ph.advertise<sensor_msgs::PointCloud2>("slam_map_downsampled", 10);
  m_mapColorShadingPublisher = ph.advertise<sensor_msgs::PointCloud2>("slam_map_color_shading", 10);

  ph.param("voxel_leaf_size", m_voxelLeafSize, 0.1);

  ph.param("filter_limit_min_x", m_filterLimitMinX, -8.f);
  ph.param("filter_limit_max_x", m_filterLimitMaxX, 8.f);
  ph.param("filter_limit_min_y", m_filterLimitMinY, -8.f);
  ph.param("filter_limit_max_y", m_filterLimitMaxY, 8.f);
  ph.param("filter_limit_min_z", m_filterLimitMinZ, -8.f);
  ph.param("filter_limit_max_z", m_filterLimitMaxZ, 8.f);

  m_processKeyframeThread = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&SlamVisualizerNodelet::processKeyframeQueue, this)));
  m_processKeyframeTransformThread = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&SlamVisualizerNodelet::processKeyframeTransformQueue, this)));
}

void SlamVisualizerNodelet::receivedKeyFrame(const mrs_laser_mapping::KeyFrameConstPtr& msg)
{
  NODELET_DEBUG_STREAM("received keyframe from timestamp " << msg->header.stamp << " at time " << ros::Time::now());
  mrs_laser_mapping::KeyFramePtr keyframe(new mrs_laser_mapping::KeyFrame(*msg));
  m_keyframeBuffer.push_front(keyframe);
}

void SlamVisualizerNodelet::receivedKeyFrameTransforms(const mrs_laser_mapping::KeyFrameTransformsConstPtr& msg)
{
  NODELET_DEBUG_STREAM("received map from timestamp " << msg->header.stamp << " at time " << ros::Time::now());
  mrs_laser_mapping::KeyFrameTransformsPtr keyframeTransform(new mrs_laser_mapping::KeyFrameTransforms(*msg));
  m_keyframeTransformBuffer.push_front(keyframeTransform);
}

void SlamVisualizerNodelet::processKeyframeQueue()
{
  while (m_running)
  {
    if (m_keyframeBuffer.size() > 3)
    {
      NODELET_WARN_STREAM("elements in buffer: " << m_keyframeBuffer.size());
    }

    mrs_laser_mapping::KeyFramePtr keyframe;
    m_keyframeBuffer.pop_back(&keyframe);
    processKeyFrame(keyframe);
  }
}

void SlamVisualizerNodelet::processKeyframeTransformQueue()
{
  while (m_running)
  {
    if (m_keyframeTransformBuffer.size() > 3)
    {
      NODELET_WARN_STREAM("elements in buffer: " << m_keyframeTransformBuffer.size());
    }

    mrs_laser_mapping::KeyFrameTransformsPtr keyframeTransform;
    m_keyframeTransformBuffer.pop_back(&keyframeTransform);
    processKeyFrameTransforms(keyframeTransform);
  }
}

void SlamVisualizerNodelet::processKeyFrame(const mrs_laser_mapping::KeyFrameConstPtr& msg)
{
  if (msg->runID != m_runID)
  {
    NODELET_INFO("SlamVisualizerNodelet: Clearing keyframes");
    m_keyFrameClouds.clear();
    m_keyFrameCloudsDownsampled.clear();
    m_keyFrameCounter = 0;
    m_runID = msg->runID;
  }

  PointCloud::Ptr keyFrameCloud(new PointCloud);
  VectorStream stream(&msg->compressedCloud);
  m_compression->decodePointCloud(stream, keyFrameCloud);

  PointCloud::Ptr keyFrameCloudDownsampled(new PointCloud);

  pcl::VoxelGrid<PointType> voxelGridFilter;
  voxelGridFilter.setInputCloud(keyFrameCloud);
  voxelGridFilter.setLeafSize(m_voxelLeafSize, m_voxelLeafSize, m_voxelLeafSize);
  voxelGridFilter.filter(*keyFrameCloudDownsampled);

  pcl::PassThrough<PointType> pass;
  pass.setInputCloud(keyFrameCloudDownsampled);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(m_filterLimitMinX, m_filterLimitMaxX);
  // pass.setFilterLimitsNegative (true);
  pass.filter(*keyFrameCloudDownsampled);

  pass.setInputCloud(keyFrameCloudDownsampled);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(m_filterLimitMinY, m_filterLimitMaxY);
  // pass.setFilterLimitsNegative (true);
  pass.filter(*keyFrameCloudDownsampled);

  pass.setInputCloud(keyFrameCloudDownsampled);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(m_filterLimitMinZ, m_filterLimitMaxZ);
  // pass.setFilterLimitsNegative (true);
  pass.filter(*keyFrameCloudDownsampled);

  pcl::StatisticalOutlierRemoval<PointType> sor;
  sor.setInputCloud(keyFrameCloudDownsampled);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*keyFrameCloudDownsampled);

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr keyFrameCloudDownsampledNormals(
      new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::copyPointCloud(*keyFrameCloudDownsampled, *keyFrameCloudDownsampledNormals);

  pcl::NormalEstimation<PointType, pcl::PointXYZRGBNormal> ne;
  ne.setInputCloud(keyFrameCloudDownsampled);
  ne.setRadiusSearch(0.25);
  ne.compute(*keyFrameCloudDownsampledNormals);

  // update members here to keep lock short
  {
    boost::unique_lock<boost::mutex> lock(m_keyframeMutex);
    if (msg->keyframeID < m_keyFrameClouds.size())
    {
      *m_keyFrameClouds[msg->keyframeID] = *keyFrameCloud;
      *m_keyFrameCloudsDownsampled[msg->keyframeID] = *keyFrameCloudDownsampled;
      *m_keyFrameCloudsDownsampledNormals[msg->keyframeID] = *keyFrameCloudDownsampledNormals;
      ROS_INFO("updated keyframe cloud");
    }
    else
    {
      m_keyFrameClouds.push_back(keyFrameCloud);
      m_keyFrameCloudsDownsampled.push_back(keyFrameCloudDownsampled);
      m_keyFrameCloudsDownsampledNormals.push_back(keyFrameCloudDownsampledNormals);
      m_keyFrameCounter++;
    }
  }

  for (size_t i = 0; i < keyFrameCloudDownsampled->size(); ++i)
  {
    m_minZ = std::min(keyFrameCloudDownsampled->points[i].z, m_minZ);
    m_maxZ = std::max(keyFrameCloudDownsampled->points[i].z, m_maxZ);
  }
}

void SlamVisualizerNodelet::processKeyFrameTransforms(const mrs_laser_mapping::KeyFrameTransformsConstPtr& msg)
{
  m_keyFrameTransforms.clear();
  for (size_t i = 0; i < msg->transforms.size(); i++)
  {
    Eigen::Affine3d nodeTransformEigen;
    tf::transformMsgToEigen(msg->transforms[i], nodeTransformEigen);
    m_keyFrameTransforms.push_back(nodeTransformEigen.matrix());
  }

  ros::Rate loopRate(50);
  while (m_keyFrameTransforms.size() > m_keyFrameCounter)
  {
    NODELET_DEBUG("m_keyFrameTransforms.size() > m_keyFrameClouds.size()");
    loopRate.sleep();
  }

  /*
   * keyframe point clouds
   */
  PointCloudPtr fullGraphCloud(new PointCloud());
  PointCloudPtr downSampledGraphCloud(new PointCloud());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downSampledColorShadedGraphCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  boost::unique_lock<boost::mutex> lock(m_keyframeMutex);

  for (size_t i = 0; i < m_keyFrameCounter; i++)
  {
    PointCloudPtr transformedCloud = PointCloudPtr(new PointCloud());
    pcl::transformPointCloud(*m_keyFrameClouds[i], *transformedCloud, m_keyFrameTransforms[i].cast<float>());

    PointCloudPtr transformedCloudDownsampled = PointCloudPtr(new PointCloud());
    pcl::transformPointCloud(*m_keyFrameCloudsDownsampled[i], *transformedCloudDownsampled,
                             m_keyFrameTransforms[i].cast<float>());

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformedCloudDownsampledNormals =
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::transformPointCloud(*m_keyFrameCloudsDownsampledNormals[i], *transformedCloudDownsampledNormals,
                             m_keyFrameTransforms[i].cast<float>());

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloudDownsampledRGB =
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

    transformedCloudDownsampledRGB->points.resize(transformedCloudDownsampled->points.size());

    float light_x = 1;
    float light_y = 0;
    float light_z = 0;

    std::vector<float> direction_of_light;
    direction_of_light.push_back(light_x);
    direction_of_light.push_back(light_y);
    direction_of_light.push_back(light_z);

    float min_v = 0.3f;

    for (size_t i = 0; i < transformedCloudDownsampled->points.size(); i++)
    {
      transformedCloudDownsampledRGB->points[i].x = transformedCloudDownsampled->points[i].x;
      transformedCloudDownsampledRGB->points[i].y = transformedCloudDownsampled->points[i].y;
      transformedCloudDownsampledRGB->points[i].z = transformedCloudDownsampled->points[i].z;

      float value_range_min = 0.0f;
      float value_range_max = 45.f;  // 360.0f;

      float h = value_range_min +
                ((transformedCloudDownsampledRGB->points[i].z - m_minZ) / (m_maxZ - m_minZ)) *
                    (value_range_max - value_range_min);
      h = std::min(360.0f, std::max(0.0f, h));

      // s = 1.f for color 0.f for grayscale?
      float s = 0.5f;

      // normalen zum viewpoint flippen
      flipNormalTowardsViewpoint(transformedCloudDownsampledNormals->points[i], direction_of_light[0],
                                 direction_of_light[1], direction_of_light[2],
                                 transformedCloudDownsampledNormals->points[i].normal_x,
                                 transformedCloudDownsampledNormals->points[i].normal_y,
                                 transformedCloudDownsampledNormals->points[i].normal_z);
      // winkel zwischen normalen und lichtvektor
      float product_of_vektors = direction_of_light[0] * transformedCloudDownsampledNormals->points[i].normal_x +
                                 direction_of_light[1] * transformedCloudDownsampledNormals->points[i].normal_y +
                                 direction_of_light[2] * transformedCloudDownsampledNormals->points[i].normal_z;
      float lenght_normal = sqrt(transformedCloudDownsampledNormals->points[i].normal_x *
                                     transformedCloudDownsampledNormals->points[i].normal_x +
                                 transformedCloudDownsampledNormals->points[i].normal_y *
                                     transformedCloudDownsampledNormals->points[i].normal_y +
                                 transformedCloudDownsampledNormals->points[i].normal_z *
                                     transformedCloudDownsampledNormals->points[i].normal_z);
      float lenght_light =
          sqrt(direction_of_light[0] * direction_of_light[0] + direction_of_light[1] * direction_of_light[1] +
               direction_of_light[2] * direction_of_light[2]);
      float angle = static_cast<float>(acos(product_of_vektors / (lenght_normal * lenght_light)));

      angle = fabs(angle);

      if (angle >= 1.570797)
      {
        float diff = angle - 1.570797;
        angle = angle - (diff * 2.0);
      }

      float v = ((1.0 - (angle / 1.6)) * (1.0 - min_v)) + min_v;

      if (std::isnan(v))
        v = 1.0f;

      // berechnete Werte n rgb umwandeln ...
      float r, g, b;
      HSVtoRGB(h, s, v, &r, &g, &b);
      r *= 255.0f;
      r = std::max(0.0f, std::min(255.0f, r));
      g *= 255.0f;
      g = std::max(0.0f, std::min(255.0f, g));
      b *= 255.0f;
      b = std::max(0.0f, std::min(255.0f, b));
      if ((r > 250.0f) && (g > 250.0f) && (b > 250.0f) && (i > 0))
      {
        PCL_INFO("color error: %d, %0.2f, %0.2f, %0.2f ... %0.2f %0.2f %0.2f\n", i,
                 transformedCloudDownsampledRGB->points[i].x, transformedCloudDownsampledRGB->points[i].y,
                 transformedCloudDownsampledRGB->points[i].z, h, s, v);
        r = transformedCloudDownsampledRGB->points[i - 1].r;
        g = transformedCloudDownsampledRGB->points[i - 1].g;
        b = transformedCloudDownsampledRGB->points[i - 1].b;
      }

      transformedCloudDownsampledRGB->points[i].r = static_cast<unsigned char>(r);
      transformedCloudDownsampledRGB->points[i].g = static_cast<unsigned char>(g);
      transformedCloudDownsampledRGB->points[i].b = static_cast<unsigned char>(b);
    }

    *fullGraphCloud += *transformedCloud;
    *downSampledGraphCloud += *transformedCloudDownsampled;
    *downSampledColorShadedGraphCloud += *transformedCloudDownsampledRGB;
  }

  fullGraphCloud->header.stamp = pcl_conversions::toPCL(msg->header.stamp);
  fullGraphCloud->header.frame_id = msg->header.frame_id;
  m_mapPublisher.publish(fullGraphCloud);

  downSampledGraphCloud->header.stamp = pcl_conversions::toPCL(msg->header.stamp);
  downSampledGraphCloud->header.frame_id = msg->header.frame_id;
  m_mapDownsampledPublisher.publish(downSampledGraphCloud);

  downSampledColorShadedGraphCloud->header.stamp = pcl_conversions::toPCL(msg->header.stamp);
  downSampledColorShadedGraphCloud->header.frame_id = msg->header.frame_id;
  m_mapColorShadingPublisher.publish(downSampledColorShadedGraphCloud);

  /*
   * edge marker
   */
  visualization_msgs::Marker points, line_list;
  points.header.frame_id = line_list.header.frame_id = msg->header.frame_id;

  points.header.stamp = line_list.header.stamp = msg->header.stamp;
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

  ROS_ASSERT(msg->edgeStartPoints.size() == msg->edgeEndPoints.size());

  for (size_t i = 0; i < msg->edgeStartPoints.size(); i++)
  {
    line_list.points.push_back(msg->edgeStartPoints[i]);
    line_list.points.push_back(msg->edgeEndPoints[i]);
  }

  for (size_t i = 0; i < m_keyFrameTransforms.size(); i++)
  {
    geometry_msgs::Point p;
    p.x = m_keyFrameTransforms[i](0, 3);
    p.y = m_keyFrameTransforms[i](1, 3);
    p.z = m_keyFrameTransforms[i](2, 3);

    points.points.push_back(p);
  }

  m_slamGraphMarkerPublisher.publish(points);
  m_slamGraphMarkerPublisher.publish(line_list);
}
}
PLUGINLIB_EXPORT_CLASS(mrs_laser_mapping::SlamVisualizerNodelet, nodelet::Nodelet)
