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
#include <pcl/io/pcd_io.h>
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
  : is_running_(true)
  , slam_map_frame_id_("/world_corrected_slam")
  , filter_limit_min_x_(-8.f)
  , filter_limit_max_x_(8.f)
  , filter_limit_min_y_(-8.f)
  , filter_limit_max_y_(8.f)
  , filter_limit_min_z_(-8.f)
  , filter_limit_max_z_(8.f)
	, run_id_(0)
  , key_frame_counter_(0)
	, min_z_(std::numeric_limits<float>::max())
  , max_z_(std::numeric_limits<float>::min())
	, keyframe_buffer_(10)
  , keyframe_transform_buffer_(1)
{
  NODELET_INFO("Initializing nodelet.. ");

//  compression_.reset(new Compression(pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR));
}

SlamVisualizerNodelet::~SlamVisualizerNodelet()
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

void SlamVisualizerNodelet::onInit()
{
  NODELET_INFO("SlamVisualizerNodelet::onInit nodelet...");

  ros::NodeHandle& ph = getMTPrivateNodeHandle();

  ph.getParam("slam_frame_id", slam_map_frame_id_);

  key_frame_subscriber_ = ph.subscribe("keyframes", 1, &SlamVisualizerNodelet::receivedKeyFrame, this);
  key_frame_transforms_subscriber_ =
      ph.subscribe("keyframe_transforms", 1, &SlamVisualizerNodelet::receivedKeyFrameTransforms, this);

  slam_graph_marker_publisher_ = ph.advertise<visualization_msgs::Marker>("slam_graph_marker", 10);
  map_publisher_ = ph.advertise<sensor_msgs::PointCloud2>("slam_map", 10);
  map_downsampled_publisher_ = ph.advertise<sensor_msgs::PointCloud2>("slam_map_downsampled", 10);
  map_color_shading_publisher_ = ph.advertise<sensor_msgs::PointCloud2>("slam_map_color_shading", 10);

  ph.param("voxel_leaf_size", voxel_leaf_size_, 0.1);

  ph.param("filter_limit_min_x", filter_limit_min_x_, -8.f);
  ph.param("filter_limit_max_x", filter_limit_max_x_, 8.f);
  ph.param("filter_limit_min_y", filter_limit_min_y_, -8.f);
  ph.param("filter_limit_max_y", filter_limit_max_y_, 8.f);
  ph.param("filter_limit_min_z", filter_limit_min_z_, -8.f);
  ph.param("filter_limit_max_z", filter_limit_max_z_, 8.f);
  
  ros::param::param<std::string>("file_name_prefix", file_name_prefix_, "visualizer");


  process_keyframe_thread_ = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&SlamVisualizerNodelet::processKeyframeQueue, this)));
  process_keyframe_transform_thread_ = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&SlamVisualizerNodelet::processKeyframeTransformQueue, this)));
}

void SlamVisualizerNodelet::receivedKeyFrame(const mrs_laser_mapping::KeyFrameConstPtr& msg)
{
  NODELET_DEBUG_STREAM("received keyframe from timestamp " << msg->header.stamp << " at time " << ros::Time::now());
  mrs_laser_mapping::KeyFramePtr keyframe(new mrs_laser_mapping::KeyFrame(*msg));
  keyframe_buffer_.push_front(keyframe);
}

void SlamVisualizerNodelet::receivedKeyFrameTransforms(const mrs_laser_mapping::KeyFrameTransformsConstPtr& msg)
{
  NODELET_DEBUG_STREAM("received transforms from timestamp " << msg->header.stamp << " at time " << ros::Time::now());
  mrs_laser_mapping::KeyFrameTransformsPtr keyframeTransform(new mrs_laser_mapping::KeyFrameTransforms(*msg));
  keyframe_transform_buffer_.push_front(keyframeTransform);
}

void SlamVisualizerNodelet::processKeyframeQueue()
{
  while (is_running_)
  {
    if (keyframe_buffer_.size() > 3)
    {
      NODELET_WARN_STREAM("elements in buffer: " << keyframe_buffer_.size());
    }

    mrs_laser_mapping::KeyFramePtr keyframe;
    keyframe_buffer_.pop_back(&keyframe);
    processKeyFrame(keyframe);
  }
}

void SlamVisualizerNodelet::processKeyframeTransformQueue()
{
  while (is_running_)
  {
    if (keyframe_transform_buffer_.size() > 3)
    {
      NODELET_WARN_STREAM("elements in buffer: " << keyframe_transform_buffer_.size());
    }

    mrs_laser_mapping::KeyFrameTransformsPtr keyframeTransform;
    keyframe_transform_buffer_.pop_back(&keyframeTransform);
    processKeyFrameTransforms(keyframeTransform);
  }
}

void SlamVisualizerNodelet::processKeyFrame(const mrs_laser_mapping::KeyFrameConstPtr& msg)
{
  pcl::StopWatch watch; 
  
  if (msg->runID != run_id_)
  {
    NODELET_INFO("SlamVisualizerNodelet: Clearing keyframes");
    key_frame_clouds_.clear();
    key_frame_clouds_downsampled_.clear();
    key_frame_counter_ = 0;
    run_id_ = msg->runID;
  }

  PointCloud::Ptr key_frame_cloud(new PointCloud);
//   VectorStream stream(&msg->compressedCloud);
//   compression_->decodePointCloud(stream, key_frame_cloud);
  
  pcl::fromROSMsg(msg->uncompressed_cloud, *key_frame_cloud);


  
  PointCloud::Ptr key_frame_cloud_downsampled(new PointCloud);

  pcl::VoxelGrid<PointType> voxel_grid_filter;
  voxel_grid_filter.setInputCloud(key_frame_cloud);
  voxel_grid_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  voxel_grid_filter.filter(*key_frame_cloud_downsampled);

  pcl::PassThrough<PointType> pass;
  pass.setInputCloud(key_frame_cloud_downsampled);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(filter_limit_min_x_, filter_limit_max_x_);
  // pass.setFilterLimitsNegative (true);
  pass.filter(*key_frame_cloud_downsampled);

  pass.setInputCloud(key_frame_cloud_downsampled);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(filter_limit_min_y_, filter_limit_max_y_);
  // pass.setFilterLimitsNegative (true);
  pass.filter(*key_frame_cloud_downsampled);

  pass.setInputCloud(key_frame_cloud_downsampled);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(filter_limit_min_z_, filter_limit_max_z_);
  // pass.setFilterLimitsNegative (true);
  pass.filter(*key_frame_cloud_downsampled);

//   pcl::StatisticalOutlierRemoval<PointType> sor;
//   sor.setInputCloud(key_frame_cloud_downsampled);
//   sor.setMeanK(50);
//   sor.setStddevMulThresh(1.5);
//   sor.filter(*key_frame_cloud_downsampled);

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr key_frame_cloud_downsampled_normals(
      new pcl::PointCloud<pcl::PointXYZRGBNormal>);
//   pcl::copyPointCloud(*key_frame_cloud_downsampled, *key_frame_cloud_downsampled_normals);
// 
//   pcl::NormalEstimation<PointType, pcl::PointXYZRGBNormal> ne;
//   ne.setInputCloud(key_frame_cloud_downsampled);
//   ne.setRadiusSearch(0.25);
//   ne.compute(*key_frame_cloud_downsampled_normals);

  // update members here to keep lock short
  {
    boost::unique_lock<boost::mutex> lock(keyframe_mutex_);
    if (msg->keyframeID < key_frame_clouds_.size())
    {
      *key_frame_clouds_[msg->keyframeID] = *key_frame_cloud;
      *key_frame_clouds_downsampled_[msg->keyframeID] = *key_frame_cloud_downsampled;
      *key_frame_clouds_downsampled_normals_[msg->keyframeID] = *key_frame_cloud_downsampled_normals;
      ROS_INFO("updated keyframe cloud");
    }
    else
    {
      key_frame_clouds_.push_back(key_frame_cloud);
      key_frame_clouds_downsampled_.push_back(key_frame_cloud_downsampled);
      key_frame_clouds_downsampled_normals_.push_back(key_frame_cloud_downsampled_normals);
      key_frame_counter_++;
    }
  }

  for (size_t i = 0; i < key_frame_cloud_downsampled->size(); ++i)
  {
    min_z_ = std::min(key_frame_cloud_downsampled->points[i].z, min_z_);
    max_z_ = std::max(key_frame_cloud_downsampled->points[i].z, max_z_);
  }
  
  PointCloud::Ptr map_alloc_cloud( new PointCloud() );
	
  for (unsigned int i = 0; i < key_frame_clouds_downsampled_.size(); i++)
  {

    PointCloudPtr transformed_cloud_downsampled = PointCloudPtr(new PointCloud());
    pcl::transformPointCloud(*key_frame_clouds_downsampled_[i], *transformed_cloud_downsampled,
                             keyframe_transforms_[i].cast<float>());
    
    (*map_alloc_cloud)+=*transformed_cloud_downsampled;
    std::stringstream name;
    
    name <<  "/tmp/" << file_name_prefix_ << "_graph_node_" << i << ".pcd";
    pcl::io::savePCDFile(name.str(), *transformed_cloud_downsampled);    
  
  }
  
  std::stringstream name;
  name << "/tmp/" << file_name_prefix_ << "_full_graph.pcd" ;
  pcl::io::savePCDFile(name.str(), *map_alloc_cloud);    
  

  
  NODELET_DEBUG_STREAM("processKeyFrame took: " << watch.getTime());
}

void SlamVisualizerNodelet::processKeyFrameTransforms(const mrs_laser_mapping::KeyFrameTransformsConstPtr& msg)
{
  keyframe_transforms_.clear();
  for (size_t i = 0; i < msg->transforms.size(); i++)
  {
    Eigen::Affine3d node_transform_eigen;
    tf::transformMsgToEigen(msg->transforms[i], node_transform_eigen);
    keyframe_transforms_.push_back(node_transform_eigen.matrix());
  }

  // if we received the new edges before the new vertex we wait till for the new vertex 
  ros::Rate rate(50);
  while (keyframe_transforms_.size() > key_frame_counter_)
  {
    NODELET_DEBUG("m_keyFrameTransforms.size() > m_keyFrameClouds.size()");
    rate.sleep();
  }

  /*
   * keyframe point clouds
   */
  PointCloudPtr graph_cloud(new PointCloud());
  PointCloudPtr downsampled_graph_cloud(new PointCloud());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  
  keyframe_mutex_.lock();

  for (size_t i = 0; i < key_frame_counter_; i++)
  {
    PointCloudPtr transformed_cloud = PointCloudPtr(new PointCloud());
    pcl::transformPointCloud(*key_frame_clouds_[i], *transformed_cloud, keyframe_transforms_[i].cast<float>());

    PointCloudPtr transformed_cloud_downsampled = PointCloudPtr(new PointCloud());
    pcl::transformPointCloud(*key_frame_clouds_downsampled_[i], *transformed_cloud_downsampled,
                             keyframe_transforms_[i].cast<float>());

//     pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformedCloudDownsampledNormals =
//         pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
//     pcl::transformPointCloud(*key_frame_clouds_downsampled_normals_[i], *transformedCloudDownsampledNormals,
//                              keyframe_transforms_[i].cast<float>());
// 
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloudDownsampledRGB =
//         pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
// 
//     transformedCloudDownsampledRGB->points.resize(transformedCloudDownsampled->points.size());
// 
//     float light_x = 1;
//     float light_y = 0;
//     float light_z = 0;
// 
//     std::vector<float> direction_of_light;
//     direction_of_light.push_back(light_x);
//     direction_of_light.push_back(light_y);
//     direction_of_light.push_back(light_z);
// 
//     float min_v = 0.3f;
// 
//     for (size_t i = 0; i < transformedCloudDownsampled->points.size(); i++)
//     {
//       transformedCloudDownsampledRGB->points[i].x = transformedCloudDownsampled->points[i].x;
//       transformedCloudDownsampledRGB->points[i].y = transformedCloudDownsampled->points[i].y;
//       transformedCloudDownsampledRGB->points[i].z = transformedCloudDownsampled->points[i].z;
// 
//       float value_range_min = 0.0f;
//       float value_range_max = 45.f;  // 360.0f;
// 
//       float h = value_range_min +
//                 ((transformedCloudDownsampledRGB->points[i].z - min_z_) / (max_z_ - min_z_)) *
//                     (value_range_max - value_range_min);
//       h = std::min(360.0f, std::max(0.0f, h));
// 
//       // s = 1.f for color 0.f for grayscale?
//       float s = 0.5f;
// 
//       // normalen zum viewpoint flippen
//       flipNormalTowardsViewpoint(transformedCloudDownsampledNormals->points[i], direction_of_light[0],
//                                  direction_of_light[1], direction_of_light[2],
//                                  transformedCloudDownsampledNormals->points[i].normal_x,
//                                  transformedCloudDownsampledNormals->points[i].normal_y,
//                                  transformedCloudDownsampledNormals->points[i].normal_z);
//       // winkel zwischen normalen und lichtvektor
//       float product_of_vektors = direction_of_light[0] * transformedCloudDownsampledNormals->points[i].normal_x +
//                                  direction_of_light[1] * transformedCloudDownsampledNormals->points[i].normal_y +
//                                  direction_of_light[2] * transformedCloudDownsampledNormals->points[i].normal_z;
//       float lenght_normal = sqrt(transformedCloudDownsampledNormals->points[i].normal_x *
//                                      transformedCloudDownsampledNormals->points[i].normal_x +
//                                  transformedCloudDownsampledNormals->points[i].normal_y *
//                                      transformedCloudDownsampledNormals->points[i].normal_y +
//                                  transformedCloudDownsampledNormals->points[i].normal_z *
//                                      transformedCloudDownsampledNormals->points[i].normal_z);
//       float lenght_light =
//           sqrt(direction_of_light[0] * direction_of_light[0] + direction_of_light[1] * direction_of_light[1] +
//                direction_of_light[2] * direction_of_light[2]);
//       float angle = static_cast<float>(acos(product_of_vektors / (lenght_normal * lenght_light)));
// 
//       angle = fabs(angle);
// 
//       if (angle >= 1.570797)
//       {
//         float diff = angle - 1.570797;
//         angle = angle - (diff * 2.0);
//       }
// 
//       float v = ((1.0 - (angle / 1.6)) * (1.0 - min_v)) + min_v;
// 
//       if (std::isnan(v))
//         v = 1.0f;
// 
//       // berechnete Werte n rgb umwandeln ...
//       float r, g, b;
//       HSVtoRGB(h, s, v, &r, &g, &b);
//       r *= 255.0f;
//       r = std::max(0.0f, std::min(255.0f, r));
//       g *= 255.0f;
//       g = std::max(0.0f, std::min(255.0f, g));
//       b *= 255.0f;
//       b = std::max(0.0f, std::min(255.0f, b));
//       if ((r > 250.0f) && (g > 250.0f) && (b > 250.0f) && (i > 0))
//       {
//         PCL_INFO("color error: %d, %0.2f, %0.2f, %0.2f ... %0.2f %0.2f %0.2f\n", i,
//                  transformedCloudDownsampledRGB->points[i].x, transformedCloudDownsampledRGB->points[i].y,
//                  transformedCloudDownsampledRGB->points[i].z, h, s, v);
//         r = transformedCloudDownsampledRGB->points[i - 1].r;
//         g = transformedCloudDownsampledRGB->points[i - 1].g;
//         b = transformedCloudDownsampledRGB->points[i - 1].b;
//       }
// 
//       transformedCloudDownsampledRGB->points[i].r = static_cast<unsigned char>(r);
//       transformedCloudDownsampledRGB->points[i].g = static_cast<unsigned char>(g);
//       transformedCloudDownsampledRGB->points[i].b = static_cast<unsigned char>(b);
//     }

    *graph_cloud += *transformed_cloud;
    *downsampled_graph_cloud += *transformed_cloud_downsampled;
//     *downSampledColorShadedGraphCloud += *transformedCloudDownsampledRGB;
  }
  
  keyframe_mutex_.unlock();

  
  graph_cloud->header.stamp = pcl_conversions::toPCL(msg->header.stamp);
  graph_cloud->header.frame_id = msg->header.frame_id;
  map_publisher_.publish(graph_cloud);

  downsampled_graph_cloud->header.stamp = pcl_conversions::toPCL(msg->header.stamp);
  downsampled_graph_cloud->header.frame_id = msg->header.frame_id;
  map_downsampled_publisher_.publish(downsampled_graph_cloud);

  downsampled_color_cloud->header.stamp = pcl_conversions::toPCL(msg->header.stamp);
  downsampled_color_cloud->header.frame_id = msg->header.frame_id;
  map_color_shading_publisher_.publish(downsampled_color_cloud);

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

  points.scale.x = 0.25;
  points.scale.y = 0.25;

  line_list.scale.x = 0.1;

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

  for (size_t i = 0; i < keyframe_transforms_.size(); i++)
  {
    geometry_msgs::Point p;
    p.x = keyframe_transforms_[i](0, 3);
    p.y = keyframe_transforms_[i](1, 3);
    p.z = keyframe_transforms_[i](2, 3);

    points.points.push_back(p);
  }

  slam_graph_marker_publisher_.publish(points);
  slam_graph_marker_publisher_.publish(line_list);
}
}
PLUGINLIB_EXPORT_CLASS(mrs_laser_mapping::SlamVisualizerNodelet, nodelet::Nodelet)
