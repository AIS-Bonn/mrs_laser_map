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

#ifndef _SURFELMAP_PUBLISHER_IMPL_H_
#define _SURFELMAP_PUBLISHER_IMPL_H_

#include <mrs_laser_mapping/surfelmap_publisher.h>

namespace mrs_laser_mapping
{
	
  SurfelMapPublisher* SurfelMapPublisher::m_instance = 0;

SurfelMapPublisher::SurfelMapPublisher() : m_nodeHandle("~"), m_lastSurfelMarkerCount(0)
{
  m_markerPublisher = m_nodeHandle.advertise<visualization_msgs::Marker>("surfels", 100);
  m_pointCloudPublisher = m_nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZ>>("pointcloud", 10);
  m_pointCloudDownsampledPublisher =
  m_nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZ>>("pointcloud_downsampled", 10);
  m_scenePointCloudPublisher = m_nodeHandle.advertise<sensor_msgs::PointCloud2>("scene_pointcloud", 10);
  m_scenePointCloudTransformedPublisher =
  m_nodeHandle.advertise<sensor_msgs::PointCloud2>("scene_pointcloud_transformed", 10);
  m_correspondencePublisher = m_nodeHandle.advertise<visualization_msgs::Marker>("correspondences", 10);
}

SurfelMapPublisher::~SurfelMapPublisher()
{
}

SurfelMapPublisher* SurfelMapPublisher::getInstance()
{
  if (m_instance == 0)
    m_instance = new SurfelMapPublisher();
  return m_instance;
}


void SurfelMapPublisher::publishTransformedScenePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  m_scenePointCloudTransformedPublisher.publish(cloud);
}
	
template <typename PointT>
void SurfelMapPublisher::publishPointCloud(const boost::shared_ptr<mrs_laser_maps::MultiResolutionalMap<PointT>>& map)
{
  if (m_pointCloudPublisher.getNumSubscribers() == 0)
    return;

  typename pcl::PointCloud<PointT>::Ptr cellPointsCloud(new pcl::PointCloud<PointT>());
  map->lock();
  map->getCellPoints(cellPointsCloud);
  map->unlock();
  cellPointsCloud->header.frame_id = map->getFrameId();
  cellPointsCloud->header.stamp = pcl_conversions::toPCL(map->getLastUpdateTimestamp());
  m_pointCloudPublisher.publish(cellPointsCloud);
  ROS_DEBUG_STREAM("publishing cell points: " << cellPointsCloud->size());
}

template <typename PointT>
void SurfelMapPublisher::publishDownsampledPointCloud(const boost::shared_ptr<mrs_laser_maps::MultiResolutionalMap<PointT>>& map,
                                                      int mapSizeDownsampled, double resolutionDownsampled,
                                                      int levelsDownsampled, int cellCapacityDownsampled)
{
  if (m_pointCloudDownsampledPublisher.getNumSubscribers() == 0)
    return;

  // publish downsampled map
  typename pcl::PointCloud<PointT>::Ptr cellPointsCloud(new pcl::PointCloud<PointT>());
  map->lock();
  map->getCellPointsDownsampled(cellPointsCloud, cellCapacityDownsampled);
  map->unlock();
  // TODO: move in map publisher to avoid duplicate pointcloud
  mrs_laser_maps::MultiResolutionalMap<PointT> mapDownsampled(mapSizeDownsampled, resolutionDownsampled, levelsDownsampled,
                                              cellCapacityDownsampled, map->getFrameId());
  mapDownsampled.addCloud(cellPointsCloud);
  typename pcl::PointCloud<PointT>::Ptr cellPointsCloudDownsampled(new pcl::PointCloud<PointT>());
  mapDownsampled.getCellPoints(cellPointsCloudDownsampled);

  cellPointsCloudDownsampled->header.frame_id = map->getFrameId();
  cellPointsCloudDownsampled->header.stamp = pcl_conversions::toPCL(map->getLastUpdateTimestamp());
  m_pointCloudDownsampledPublisher.publish(cellPointsCloudDownsampled);
}

void SurfelMapPublisher::publishScenePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  m_scenePointCloudPublisher.publish(cloud);
}

template <typename PointT>
void SurfelMapPublisher::publishCorrespondences(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source,
                                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr target,
                                                const boost::shared_ptr<mrs_laser_maps::MultiResolutionalMap<PointT>>& map)
{
  if (m_correspondencePublisher.getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker marker;
  marker.header.frame_id = map->getFrameId();
  marker.header.stamp = map->getLastUpdateTimestamp();
  marker.ns = "linelist";
  marker.id = 0;
  marker.type = marker.LINE_LIST;
  marker.action = marker.ADD;
  marker.pose.position.x = 0.f;
  marker.pose.position.y = 0.f;
  marker.pose.position.z = 0.f;
  marker.pose.orientation.w = 1.f;
  marker.pose.orientation.x = 0.f;
  marker.pose.orientation.y = 0.f;
  marker.pose.orientation.z = 0.f;
  marker.scale.x = 0.01f;
  marker.scale.y = 0.01f;
  marker.scale.z = 0.01f;
  //	marker.color.a = 1.0;

  // Create the vertices for the points and lines
  for (size_t i = 0; i < source->points.size(); ++i)
  {
    geometry_msgs::Point p1;
    p1.x = source->points[i].x;
    p1.y = source->points[i].y;
    p1.z = source->points[i].z;

    geometry_msgs::Point p2;
    p2.x = target->points[i].x;
    p2.y = target->points[i].y;
    p2.z = target->points[i].z;

    marker.points.push_back(p1);
    marker.points.push_back(p2);

    std_msgs::ColorRGBA col;

    col.r = source->points[i].r / 255.f;
    col.g = source->points[i].g / 255.f;
    col.b = source->points[i].b / 255.f;
    col.a = 1.f;

    marker.colors.push_back(col);
    marker.colors.push_back(col);
  }

  m_correspondencePublisher.publish(marker);
}

template <typename PointT>
void SurfelMapPublisher::publishSurfelMarkers(const boost::shared_ptr<mrs_laser_maps::MultiResolutionalMap<PointT>>& map)
{
  if (m_markerPublisher.getNumSubscribers() == 0)
    return;

  unsigned int markerId = 0;
  std::vector<float> cellSizes;
  typename mrs_laser_maps::MultiResolutionalMap<PointT>::AlignedCellVectorVector occupiedCells;
  std::vector<std::vector<pcl::PointXYZ>> occupiedCellsCenters;

  int levels = map->getLevels();
  for (unsigned int i = 0; i < m_lastSurfelMarkerCount; ++i)
  {
    for (unsigned int l = 0; l < levels; l++)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = map->getFrameId();
      marker.header.stamp = map->getLastUpdateTimestamp();
      marker.ns = boost::lexical_cast<std::string>(l);
      marker.id = i;
      marker.type = marker.SPHERE;
      marker.action = marker.DELETE;
      m_markerPublisher.publish(marker);
    }
  }

  for (unsigned int l = 0; l < levels; l++)
  {
    cellSizes.push_back(map->getCellSize(l));

    typename mrs_laser_maps::MultiResolutionalMap<PointT>::AlignedCellVector occupiedCellsTemp;
    std::vector<pcl::PointXYZ> occupiedCellsCentersTemp;
    map->getOccupiedCells(occupiedCellsTemp, l);
    map->getOccupiedCells(occupiedCellsCentersTemp, l);

    occupiedCells.push_back(occupiedCellsTemp);
    occupiedCellsCenters.push_back(occupiedCellsCentersTemp);
  }

  for (unsigned int l = 0; l < cellSizes.size(); l++)
  {
    float cellSize = cellSizes[l];

    for (size_t i = 0; i < occupiedCells[l].size(); i++)
    {
      const mrs_laser_maps::Surfel& surfel = occupiedCells[l][i].surfel_;

      //				if (surfel.num_points_ < 15 )
      //					continue;
      //
      //				if ( surfel.unevaluated_ ) {
      //					ROS_ERROR("not unevaluated surfel");
      //					continue;
      //				}

      Eigen::Matrix<double, 3, 3> cov = surfel.cov_.block(0, 0, 3, 3);
      Eigen::EigenSolver<Eigen::Matrix3d> solver(cov);
      Eigen::Matrix<double, 3, 3> eigenvectors = solver.eigenvectors().real();
      //				double eigenvalues[3];
      Eigen::Matrix<double, 3, 1> eigenvalues = solver.eigenvalues().real();
      //				for(int j = 0; j < 3; ++j) {
      //					Eigen::Matrix<double, 3, 1> mult = cov * eigenvectors.col(j);
      //					eigenvalues[j] = mult(0,0) / eigenvectors.col(j)(0);
      //				}
      if (eigenvectors.determinant() < 0)
      {
        eigenvectors.col(0)(0) = -eigenvectors.col(0)(0);
        eigenvectors.col(0)(1) = -eigenvectors.col(0)(1);
        eigenvectors.col(0)(2) = -eigenvectors.col(0)(2);
      }
      Eigen::Quaternion<double> q = Eigen::Quaternion<double>(eigenvectors);
      visualization_msgs::Marker marker;
      marker.header.frame_id = map->getFrameId();
      marker.header.stamp = map->getLastUpdateTimestamp();
      marker.ns = boost::lexical_cast<std::string>(l);
      marker.id = markerId++;
      marker.type = marker.SPHERE;
      marker.action = marker.ADD;
      marker.pose.position.x = occupiedCellsCenters[l][i].x - cellSize / 2 + surfel.mean_(0);
      marker.pose.position.y = occupiedCellsCenters[l][i].y - cellSize / 2 + surfel.mean_(1);
      marker.pose.position.z = occupiedCellsCenters[l][i].z - cellSize / 2 + surfel.mean_(2);
      marker.pose.orientation.w = q.w();
      marker.pose.orientation.x = q.x();
      marker.pose.orientation.y = q.y();
      marker.pose.orientation.z = q.z();
      marker.scale.x = std::max(sqrt(eigenvalues[0]) * 3, 0.01);
      marker.scale.y = std::max(sqrt(eigenvalues[1]) * 3, 0.01);
      marker.scale.z = std::max(sqrt(eigenvalues[2]) * 3, 0.01);
      marker.color.a = 1.0;

      double dot = surfel.normal_.dot(Eigen::Vector3d(0., 0., 1.));
      if (surfel.normal_.norm() > 1e-10)
        dot /= surfel.normal_.norm();
      double angle = acos(fabs(dot));
      double angle_normalized = 2. * angle / M_PI;
      marker.color.r = ColorMapJet::red(angle_normalized);  // fabs(surfel.normal_(0));
      marker.color.g = ColorMapJet::green(angle_normalized);
      marker.color.b = ColorMapJet::blue(angle_normalized);

      m_markerPublisher.publish(marker);
    }
  }
  m_lastSurfelMarkerCount = markerId;
}
}

#endif
