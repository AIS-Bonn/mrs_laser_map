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

#ifndef _MAP_PUBLISHER_IMPL_H_
#define _MAP_PUBLISHER_IMPL_H_

#include <mrs_laser_mapping/map_publisher.h>

#include <pcl/common/time.h>

namespace mrs_laser_mapping
{
template <typename PointT>
void MapPublisher::publishOccupiedCells(const boost::shared_ptr<mrs_laser_maps::MultiResolutionalMap<PointT>>& map)
{
  if (m_markerPublisher.getNumSubscribers() == 0)
    return;

  int cubenr = 0;
  int markerId = 0;
  std::vector<float> cellSizes;
  typename mrs_laser_maps::MultiResolutionalMap<PointT>::AlignedCellVectorVector occupiedCells;
  std::vector<std::vector<pcl::PointXYZ>> occupiedCellsCenters;

  int levels = map->getLevels();

  /*
  * log odds occupancy to [0,1]
  */
  // float occupancyScale = fabs(map->getClampingThreshMin()) + fabs(map->getClampingThreshMax());
  // float occupancyOffset = fabs(map->getClampingThreshMin());

  for (int l = 0; l < levels; l++)
  {
    cellSizes.push_back(map->getCellSize(l));

    typename mrs_laser_maps::MultiResolutionalMap<PointT>::AlignedCellVector occupiedCellsTemp;

    std::vector<pcl::PointXYZ> occupiedCellsCentersTemp;
    map->getOccupiedCells(occupiedCellsTemp, l);
    map->getOccupiedCells(occupiedCellsCentersTemp, l);

    occupiedCells.push_back(occupiedCellsTemp);
    occupiedCellsCenters.push_back(occupiedCellsCentersTemp);
  }

  for (int l = cellSizes.size() - 1; l >= 0; l--)
  {
    float cellSize = cellSizes[l];

    std::string cubeName = boost::lexical_cast<std::string>(cubenr++);  // std::to_string(i);

    Eigen::Quaternionf q;
    visualization_msgs::Marker marker;
    marker.header.frame_id = map->getFrameId();
    marker.header.stamp = map->getLastUpdateTimestamp();
    marker.ns = "marker_test";
    marker.id = markerId++;
    marker.lifetime = ros::Duration(0);
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = cellSize;
    marker.scale.y = cellSize;
    marker.scale.z = cellSize;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.1f;  // std::max( std::min(m_map[ iz ][ iy ][ ix ], 1.f), 0.f);
    marker.pose.orientation.x = 0.f;
    marker.pose.orientation.y = 0.f;
    marker.pose.orientation.z = 0.f;
    marker.pose.orientation.w = 1.f;
    marker.points.resize(occupiedCells[l].size());
    marker.colors.resize(occupiedCells[l].size());

    for (size_t i = 0; i < occupiedCells[l].size(); i++)
    {
      marker.points[i].x = occupiedCellsCenters[l][i].x;  // - cellSize/2.f;
      marker.points[i].y = occupiedCellsCenters[l][i].y;  // - cellSize/2.f;
      marker.points[i].z = occupiedCellsCenters[l][i].z;  // - cellSize/2.f;

      double occupancyNorm =
          static_cast<double>(l) / static_cast<double>(cellSizes.size());  //(occupiedCells[l][i].getOccupancy() +
                                                                           //occupancyOffset) / occupancyScale;

      marker.colors[i].r = std::min(1.0, occupancyNorm);  // 0.0;
      marker.colors[i].g = 1.f;
      marker.colors[i].b = 1.f - std::min(1.0, occupancyNorm);

      // 			marker.colors[ i ].r = std::min( 1.0, occupancyNorm );//0.0;
      // 			marker.colors[ i ].g = 1.f;
      // 			marker.colors[ i ].b = 1.f - std::min( 1.0, occupancyNorm );

      marker.colors[i].a = 0.5f;  // 0.075f; //std::max( std::min(m_map[ iz ][ iy ][ ix ], 1.f), 0.f);
      //			if (cut ( marker.points[ i ].x, marker.points[ i ].y, marker.points[ i ].z ) )
      //				marker.colors[ i ].a = 0.f;
    }
    m_markerPublisher.publish(marker);
  }
}

template <typename PointT>
void MapPublisher::publishCellsWithOccupancy(const boost::shared_ptr<mrs_laser_maps::MultiResolutionalMap<PointT>>& map)
{
  if (m_markerPublisher.getNumSubscribers() == 0)
    return;

  int cubenr = 0;
  int markerId = 0;
  std::vector<float> cellSizes;
  std::vector<std::vector<Eigen::Vector3f>> occupiedCellOffsets;
  typename mrs_laser_maps::MultiResolutionalMap<PointT>::AlignedCellVectorVector occupiedGridCells;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cellPointsCloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr testCloud(new pcl::PointCloud<pcl::PointXYZ>);
  map->getCellPoints(testCloud);

  int levels = map->getLevels();

  for (int l = 0; l < levels; l++)
  {
    cellSizes.push_back(map->getCellSize(l));

    std::vector<Eigen::Vector3f> occupiedCellsTemp;
    typename mrs_laser_maps::MultiResolutionalMap<PointT>::AlignedCellVector occupiedGridCellsTemp;

    map->getOccupiedCellsWithOffset(occupiedGridCellsTemp, occupiedCellsTemp, l);
    occupiedCellOffsets.push_back(occupiedCellsTemp);
    occupiedGridCells.push_back(occupiedGridCellsTemp);
  }

  for (int l = cellSizes.size() - 1; l >= 0; l--)
  {
    float cellSize = cellSizes[l];

    Eigen::Quaternionf q;
    visualization_msgs::Marker marker;
    marker.header.frame_id = map->getFrameId();
    marker.header.stamp = map->getLastUpdateTimestamp();
    marker.ns = "marker_test_occupancy_information";
    marker.id = markerId++;
    marker.lifetime = ros::Duration(0);
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = cellSize;
    marker.scale.y = cellSize;
    marker.scale.z = cellSize;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.1f;  // std::max( std::min(m_map[ iz ][ iy ][ ix ], 1.f), 0.f);
    marker.pose.orientation.x = 0.f;
    marker.pose.orientation.y = 0.f;
    marker.pose.orientation.z = 0.f;
    marker.pose.orientation.w = 1.f;
    marker.points.resize(occupiedCellOffsets[l].size());
    marker.colors.resize(occupiedCellOffsets[l].size());

    for (size_t i = 0; i < occupiedCellOffsets[l].size(); i++)
    {
      // if ( occupiedGridCells[l][i].m_debugState == mrs_laser_maps::GridCellWithStatistics::FREE )
      //	continue;

      geometry_msgs::Point p;
      p.x = occupiedCellOffsets[l][i](0);  // - cellSize/2.f;
      p.y = occupiedCellOffsets[l][i](1);  // - cellSize/2.f;
      p.z = occupiedCellOffsets[l][i](2);  // - cellSize/2.f;

      std_msgs::ColorRGBA c;

      switch (occupiedGridCells[l][i].m_debugState)
      {
		case mrs_laser_maps::MultiResolutionalMap<PointT>::GridCellType::UNINITIALIZED:
          c.r = 0.7f;
          c.g = 0.7f;
          c.b = 0.7f;
          c.a = 0.1f;
          break;
		case mrs_laser_maps::MultiResolutionalMap<PointT>::GridCellType::FREE:
          c.r = 0.f;
          c.g = 1.f;
          c.b = 1.f;
          c.a = 0.f;
          break;

		case mrs_laser_maps::MultiResolutionalMap<PointT>::GridCellType::OCCUPIED:
          c.r = 0.f;
          c.g = 1.f;
          c.b = 1.f;
          c.a = 1.f;
          break;

		case mrs_laser_maps::MultiResolutionalMap<PointT>::GridCellType::UNKNOWN:
          c.r = 0.7f;
          c.g = 0.7f;
          c.b = 0.7f;
          c.a = 0.5f;
          break;

        default:
          c.r = 1.f;
          c.g = 0.0f;
          c.b = 0.0f;
          c.a = 0.5f;
          break;
      }

      marker.points.push_back(p);
      marker.colors.push_back(c);
    }
    m_markerPublisher.publish(marker);
  }
}

template <typename PointT>
void MapPublisher::publishMap(const boost::shared_ptr<mrs_laser_maps::MultiResolutionalMap<PointT>>& map)
{
  if (m_mapMsgPublisher.getNumSubscribers() == 0)
    return;
  pcl::StopWatch timer;
  timer.reset();

  mrs_laser_mapping::MultiResolutionMapPtr mapMsg(new mrs_laser_mapping::MultiResolutionMap);
  mapMsg->header.stamp = map->getLastUpdateTimestamp();
  mapMsg->header.frame_id = map->getFrameId();
  mapMsg->levels = map->getLevels();
  mapMsg->size_in_meters = map->getSizeInMeters();
  mapMsg->resolution = map->getResolution();
  mapMsg->cell_capacity = map->getCellCapacity();

  unsigned int nrPoints = 0;
  for (unsigned int i = 0; i < map->getLevels(); i++)
  {
    typename pcl::PointCloud<PointT>::Ptr points(new pcl::PointCloud<PointT>());
    map->getCellPoints(points, i);

    sensor_msgs::PointCloud2 rosCloud;

    pcl::toROSMsg(*points, rosCloud);

    mapMsg->cloud.push_back(rosCloud);
    nrPoints += points->size();
  }

  /*
   * get all free cells
   */
  std::vector<float> cellSizes;
  std::vector<std::vector<Eigen::Vector3f>> occupiedCellOffsets;
  typename mrs_laser_maps::MultiResolutionalMap<PointT>::CellPointerVectorVector occupiedGridCells;
  int levels = map->getLevels();

  for (int l = 0; l < levels; l++)
  {
    cellSizes.push_back(map->getCellSize(l));

    std::vector<Eigen::Vector3f> occupiedCellsTemp;
    typename mrs_laser_maps::MultiResolutionalMap<PointT>::CellPointerVector occupiedGridCellsTemp;

    map->getCellsWithOffset(occupiedGridCellsTemp, occupiedCellsTemp, l, -99999.f);  // TODO: this sucks...

    occupiedCellOffsets.push_back(occupiedCellsTemp);
    occupiedGridCells.push_back(occupiedGridCellsTemp);
  }

  for (int l = cellSizes.size() - 1; l >= 0; l--)
  {
    for (size_t i = 0; i < occupiedCellOffsets[l].size(); i++)
    {
      if ((*occupiedGridCells[l][i]).getOccupancy() <= 0.f)
      {  // thos
         // 				if ( occupiedGridCells[l][i]->m_debugState == mrs_laser_maps::GridCellWithStatistics::FREE ) {
        geometry_msgs::Point p;
        p.x = occupiedCellOffsets[l][i](0);
        p.y = occupiedCellOffsets[l][i](1);
        p.z = occupiedCellOffsets[l][i](2);

        mapMsg->free_cell_centers.push_back(p);
      }
    }
  }

  m_mapMsgPublisher.publish(mapMsg);
  ROS_DEBUG_STREAM_NAMED("map_publisher","published map message took: " << timer.getTime() << " ms with " << nrPoints
                                                 << " points. from timestamp " << mapMsg->header.stamp << " at time "
                                                 << ros::Time::now());
}

template <typename PointT>
void MapPublisher::publishMapLevelColor(const boost::shared_ptr<mrs_laser_maps::MultiResolutionalMap<PointT>>& map)
{
  if (m_levelColorCellPublisher.getNumSubscribers() == 0)
    return;

  typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr cellPointsCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  for (size_t l = 0; l < map->getLevels(); ++l)
  {
    double levelNumberNormalized = (static_cast<double>(l + 1) / map->getLevels());

    typename pcl::PointCloud<PointT>::Ptr levelPointsCloud(new pcl::PointCloud<PointT>());
    map->getCellPoints(levelPointsCloud, l);

    float r, g, b;
    ColorMapJet::getRainbowColor(levelNumberNormalized, r, g, b);

    for (size_t i = 0; i < levelPointsCloud->points.size(); ++i)
    {
      pcl::PointXYZRGB p;
      p.x = levelPointsCloud->points[i].x;
      p.y = levelPointsCloud->points[i].y;
      p.z = levelPointsCloud->points[i].z;

      p.r = static_cast<int>(255.f * r);
      p.g = static_cast<int>(255.f * g);
      p.b = static_cast<int>(255.f * b);

      cellPointsCloud->points.push_back(p);
    }
  }

  cellPointsCloud->header.frame_id = map->getFrameId();
  cellPointsCloud->header.stamp = pcl_conversions::toPCL(map->getLastUpdateTimestamp());
  m_levelColorCellPublisher.publish(cellPointsCloud);
}
}

#endif
