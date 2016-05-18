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

#include <mrs_laser_mapping/graph_publisher.h>

#include <random>

#include <pcl_ros/transforms.h>

#include <eigen_conversions/eigen_msg.h>

#include <tf_conversions/tf_eigen.h>

#include <visualization_msgs/Marker.h>

#include <mrs_laser_mapping/KeyFrame.h>
#include <mrs_laser_mapping/KeyFrameTransforms.h>


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

template <typename MapPointType, typename MapType>  
GraphPublisher<MapPointType, MapType>* GraphPublisher<MapPointType, MapType>::instance_ = 0;

template <typename MapPointType, typename MapType>
GraphPublisher<MapPointType, MapType>::GraphPublisher() 
: node_counter_(0)
, start_time_(ros::Time::now())
{
  ros::NodeHandle node_handle("~");
  ros::param::param<std::string>("slam_frame_id", frame_id_slam_map_, "/world_corrected_slam");
  ros::param::param<std::string>("odom_frame_id", frame_id_odometry_, "/odom");
  
  pub_slam_graph_marker_ = node_handle.advertise<visualization_msgs::Marker>("slam_graph_marker", 10);
  pub_map_ = node_handle.advertise<sensor_msgs::PointCloud2>("slam_map", 10);
  pub_map_downsampled_ = node_handle.advertise<sensor_msgs::PointCloud2>("slam_map_downsampled", 10);
  pub_reference_keyframe_ = node_handle.advertise<sensor_msgs::PointCloud2>("slam_reference_keyframe", 10);
  pub_keyframes_ = node_handle.advertise<mrs_laser_mapping::KeyFrame>("keyframes", 1);
  pub_keyframe_transforms_ = node_handle.advertise<mrs_laser_mapping::KeyFrameTransforms>("keyframe_transforms", 1);
  pub_odometry_ = node_handle.advertise<nav_msgs::Odometry>("odometry", 1);
  pub_local_map_ = node_handle.advertise<sensor_msgs::PointCloud2>("m_local_map", 10);
 
  
  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_int_distribution<uint32_t> distribution;

  run_id_ = distribution(mt);
}

template <typename MapPointType, typename MapType>
GraphPublisher<MapPointType, MapType>::~GraphPublisher()
{
}

template <typename MapPointType, typename MapType>
GraphPublisher<MapPointType, MapType>* GraphPublisher<MapPointType, MapType>::getInstance()
{
  if (instance_ == 0)
    instance_ = new GraphPublisher<MapPointType, MapType>();
  return instance_;
}

template <typename MapPointType, typename MapType>
void GraphPublisher<MapPointType, MapType>::publishOdometryGraph(GraphPtr graph, nav_msgs::Odometry odom_msg )
{
  
  pub_odometry_.publish(odom_msg);

  odometry_msgs_.push_back(odom_msg);
  
  if ( pub_slam_graph_marker_.getNumSubscribers() == 0 )
     return; 
  
  visualization_msgs::Marker points, line_list, cylinders, innerCylinders;
  points.header.frame_id = line_list.header.frame_id = cylinders.header.frame_id = innerCylinders.header.frame_id =
      frame_id_slam_map_; 

  points.header.stamp = line_list.header.stamp = cylinders.header.stamp = innerCylinders.header.stamp =
      graph->graph_nodes_[0]->map_->getLastUpdateTimestamp();
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
  for (unsigned int i = 1; i < odometry_msgs_.size(); i++)
  {
    geometry_msgs::Point p = odometry_msgs_[i].pose.pose.position;

    Eigen::Vector3d lastPointEigen(lastPoint.x, lastPoint.y, lastPoint.z);
    Eigen::Vector3d pointEigen(p.x, p.y, p.z);

    if ((i > 1) && (pointEigen - lastPointEigen).norm() < cylinders.scale.x)
      continue;

    zOverlay = 0.0;
    cylinders.pose = odometry_msgs_[i].pose.pose;
    zOverlay += zOverlayOffset;
    cylinders.pose.position.z += zOverlay;
    cylinders.id = markerId++;
    pub_slam_graph_marker_.publish(cylinders);

    innerCylinders.pose = odometry_msgs_[i].pose.pose;

    zOverlay += zOverlayOffset;
    innerCylinders.pose.position.z += zOverlay;
    innerCylinders.id = markerId++;

    // time delta
    ros::Duration duration = odometry_msgs_[i].header.stamp - start_time_;

    float r, g, b;
    ColorMapJet::getRainbowColor((duration.toSec() / (60 * 20)), r, g, b);

    innerCylinders.color.r = r;
    innerCylinders.color.g = g;
    innerCylinders.color.b = b;
    innerCylinders.color.a = 1.0f;

    pub_slam_graph_marker_.publish(innerCylinders);

    /*
    geometry_msgs::Pose p2Pose = odom_msg[i].pose.pose;
    Eigen::Affine3d p2Eigen;
    tf::poseMsgToEigen( p2Pose, p2Eigen);
    */
    Eigen::Matrix4d p2EigenMatrix = Eigen::Matrix4d::Identity();  // p2Eigen.matrix();

    Eigen::Quaterniond q(odometry_msgs_[i].pose.pose.orientation.w, odometry_msgs_[i].pose.pose.orientation.x,
                         odometry_msgs_[i].pose.pose.orientation.y, odometry_msgs_[i].pose.pose.orientation.z);

    p2EigenMatrix.block<3, 3>(0, 0) = q.matrix();

    // 			Eigen::Vector4d pEigen(cylinders.scale.x/4, 0.0, 0.0, 1.0);
    //  			pEigen = p2EigenMatrix * pEigen ;
    // 			p.x += pEigen(0);
    // 			p.y += pEigen(1);
    // 			p.z += pEigen(2);
    //

    Eigen::Vector4d p2Eigen(cylinders.scale.x * 0.5, 0.0, 0.0, 1.0);
    p2Eigen = p2EigenMatrix * p2Eigen;

    geometry_msgs::Point p2 = odometry_msgs_[i].pose.pose.position;
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
  pub_slam_graph_marker_.publish(points);
  pub_slam_graph_marker_.publish(line_list);
}

template <typename MapPointType, typename MapType>
void GraphPublisher<MapPointType, MapType>::publishSLAMGraph(GraphPtr graph)
{
  
  if ( pub_map_.getNumSubscribers() == 0 && pub_slam_graph_marker_.getNumSubscribers() == 0 && pub_map_downsampled_.getNumSubscribers() == 0 && pub_reference_keyframe_ == 0 )
      return; 
  
  PointCloudPtr cloud_final =
      PointCloudPtr(new PointCloud());
  PointCloudPtr cloudFinalDownsampled =
      PointCloudPtr(new PointCloud());

  visualization_msgs::Marker points, line_list;
  points.header.frame_id = line_list.header.frame_id =
      frame_id_slam_map_;  // slam->keyFrames_[ 0 ]->map_->getFrameId(); // "/slam_frame"; //

  points.header.stamp = line_list.header.stamp = graph->graph_nodes_[0]->map_->getLastUpdateTimestamp();
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

  PointCloudPtr referenceKeyframeCloud =
      PointCloudPtr(new PointCloud());



  for (unsigned int i = 0; i < graph->graph_nodes_.size(); i++)
  {
    g2o::VertexSE3* v_curr = dynamic_cast<g2o::VertexSE3*>(graph->optimizer_->vertex(graph->graph_nodes_[i]->node_id_));

    // add coordinate frames for vertices
    Eigen::Matrix4d camTransform = v_curr->estimate().matrix();

    geometry_msgs::Point p;
    p.x = camTransform(0, 3);
    p.y = camTransform(1, 3);
    p.z = camTransform(2, 3);

    points.points.push_back(p);

    typename MapType::Ptr node_map_ptr =  boost::dynamic_pointer_cast<MapType> (graph->graph_nodes_[i]->map_);


    // publish downsampled map
    PointCloudPtr cellPointsCloudDownsampled(new PointCloud());
    node_map_ptr ->getCellPointsDownsampled(cellPointsCloudDownsampled, 100);
    PointCloudPtr transformedCloudDownsampled =
        PointCloudPtr(new PointCloud());
    pcl::transformPointCloud(*cellPointsCloudDownsampled, *transformedCloudDownsampled, camTransform.cast<float>());

    if (i == graph->reference_node_id_)
    {
      pcl::copyPointCloud(*transformedCloudDownsampled, *referenceKeyframeCloud );
    }
    
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

    PointCloudPtr cellPointsCloud(new PointCloud());
    node_map_ptr ->getCellPoints(cellPointsCloud);
    PointCloudPtr transformedCloud =
        PointCloudPtr(new PointCloud());
    pcl::transformPointCloud(*cellPointsCloud, *transformedCloud, camTransform.cast<float>());

    *cloud_final += *transformedCloud;
  }

  /*
   * graph edges
   */
  for (EdgeSet::iterator it = graph->optimizer_->edges().begin(); it != graph->optimizer_->edges().end(); ++it)
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

  pub_slam_graph_marker_.publish(points);
  pub_slam_graph_marker_.publish(line_list);

  cloud_final->header.stamp = pcl_conversions::toPCL(graph->graph_nodes_[0]->map_->getLastUpdateTimestamp());
  cloud_final->header.frame_id = frame_id_slam_map_;  // slam->keyFrames_[ 0 ]->map_->getFrameId();
  pub_map_.publish(cloud_final);

  cloudFinalDownsampled->header.stamp = pcl_conversions::toPCL(graph->graph_nodes_[0]->map_->getLastUpdateTimestamp());
  cloudFinalDownsampled->header.frame_id = frame_id_slam_map_;  // slam->keyFrames_[ 0 ]->map_->getFrameId();
  pub_map_downsampled_.publish(cloudFinalDownsampled);

  referenceKeyframeCloud->header.stamp = pcl_conversions::toPCL(graph->graph_nodes_[0]->map_->getLastUpdateTimestamp());
  referenceKeyframeCloud->header.frame_id = frame_id_slam_map_;  // slam->keyFrames_[ 0 ]->map_->getFrameId();
  pub_reference_keyframe_.publish(referenceKeyframeCloud);
}

template <typename MapPointType, typename MapType>
void GraphPublisher<MapPointType, MapType>::publishGraph(GraphPtr graph)
{
  if ( pub_keyframe_transforms_.getNumSubscribers() == 0)
    return;
  
  /*
   * publish every keyframe that has not been published yet
   */
  for (unsigned int i = node_counter_; i < graph->graph_nodes_.size(); i++)
  {
    MapPtr node_map_ptr =  boost::dynamic_pointer_cast<MapType> (graph->graph_nodes_[i]->map_);
    PointCloudPtr cloud_cell_points(new PointCloud());
    node_map_ptr->getCellPoints(cloud_cell_points);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cell_points_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::copyPointCloud(*cloud_cell_points, *cloud_cell_points_rgb);

    mrs_laser_mapping::KeyFrame key_frame_msg;
    key_frame_msg.header.stamp = graph->graph_nodes_[0]->map_->getLastUpdateTimestamp();;
    key_frame_msg.header.frame_id = frame_id_slam_map_;
    key_frame_msg.runID = run_id_;
    key_frame_msg.keyframeID = node_counter_++;

    Compression compression;
    VectorStream stream(&key_frame_msg.compressedCloud);
    compression.encodePointCloud(cloud_cell_points_rgb, stream);

    pub_keyframes_.publish(key_frame_msg);

//     // if a new keyframe was added, let the thread calculate the point drift
//     m_evaluatePointDrift = true;
  }	

  /*
   * publish transforms for each key frame and graph edges on each update (change for every optimizer run)
   */
  mrs_laser_mapping::KeyFrameTransforms transforms_msg;
  transforms_msg.header.stamp = graph->graph_nodes_[0]->map_->getLastUpdateTimestamp();;
  transforms_msg.header.frame_id = frame_id_slam_map_;

  // key frame transforms
  for (unsigned int i = 0; i < graph->graph_nodes_.size(); i++)
  {
    g2o::VertexSE3* v_curr = dynamic_cast<g2o::VertexSE3*>(graph->optimizer_->vertex(graph->graph_nodes_[i]->node_id_));

    Eigen::Matrix4d node_transform = v_curr->estimate().matrix();

    geometry_msgs::Transform node_transform_msg;
    tf::transformEigenToMsg(Eigen::Affine3d(node_transform), node_transform_msg);
    transforms_msg.transforms.push_back(node_transform_msg);
  }

  // graph edges
  for (EdgeSet::iterator it = graph->optimizer_->edges().begin(); it != graph->optimizer_->edges().end(); ++it)
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

    transforms_msg.edgeStartPoints.push_back(p1);
    transforms_msg.edgeEndPoints.push_back(p2);
  }

  pub_keyframe_transforms_.publish(transforms_msg);
}


}
