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
template <typename PointType, typename MapType>
void MapPublisher::publishOccupiedCells(const boost::shared_ptr<MapType>& map)
{
  if (pub_marker_.getNumSubscribers() == 0)
    return;

  int marker_id = 0;
  std::vector<float> cell_sizes;
  typename MapType::AlignedCellVectorVector occupied_cells;
  std::vector<std::vector<pcl::PointXYZ>> occupied_cell_centers;

  int levels = map->getLevels();

  for (int l = 0; l < levels; l++)
  {
    cell_sizes.push_back(map->getCellSize(l));

    typename MapType::AlignedCellVector occupied_cells_level;

    std::vector<pcl::PointXYZ> occupied_cell_centers_level;
    map->getOccupiedCells(occupied_cells_level, l);
    map->getOccupiedCells(occupied_cell_centers_level, l);

    occupied_cells.push_back(occupied_cells_level);
    occupied_cell_centers.push_back(occupied_cell_centers_level);
  }

  for (int l = cell_sizes.size() - 1; l >= 0; l--)
  {
    float cell_size = cell_sizes[l];

    Eigen::Quaternionf q;
    visualization_msgs::Marker marker;
    marker.header.frame_id = map->getFrameId();
    marker.header.stamp = map->getLastUpdateTimestamp();
    marker.ns = "marker_test";
    marker.id = marker_id++;
    marker.lifetime = ros::Duration(0);
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = cell_size;
    marker.scale.y = cell_size;
    marker.scale.z = cell_size;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.1f;  // std::max( std::min(m_map[ iz ][ iy ][ ix ], 1.f), 0.f);
    marker.pose.orientation.x = 0.f;
    marker.pose.orientation.y = 0.f;
    marker.pose.orientation.z = 0.f;
    marker.pose.orientation.w = 1.f;
    marker.points.resize(occupied_cells[l].size());
    marker.colors.resize(occupied_cells[l].size());

    for (size_t i = 0; i < occupied_cells[l].size(); i++)
    {
      marker.points[i].x = occupied_cell_centers[l][i].x;  // - cellSize/2.f;
      marker.points[i].y = occupied_cell_centers[l][i].y;  // - cellSize/2.f;
      marker.points[i].z = occupied_cell_centers[l][i].z;  // - cellSize/2.f;

      double occupancy_normalized =
          static_cast<double>(l) / static_cast<double>(cell_sizes.size());  

      marker.colors[i].r = std::min(1.0, occupancy_normalized);  // 0.0;
      marker.colors[i].g = 1.f;
      marker.colors[i].b = 1.f - std::min(1.0, occupancy_normalized);
      marker.colors[i].a = 0.5f;  
    }
    pub_marker_.publish(marker);
  }
}

template <typename PointType, typename MapType>
void MapPublisher::publishCellsWithOccupancy(const boost::shared_ptr<MapType>& map)
{
  if (pub_marker_.getNumSubscribers() == 0)
    return;

  int marker_id = 0;
  std::vector<float> cell_sizes;
  std::vector<std::vector<Eigen::Vector3f>> occupied_cell_offsets;
  typename MapType::AlignedCellVectorVector occupied_grid_cells;
  
  int levels = map->getLevels();

  for (int l = 0; l < levels; l++)
  {
    cell_sizes.push_back(map->getCellSize(l));

    std::vector<Eigen::Vector3f> occupied_cells_level;
    typename MapType::AlignedCellVector occupied_grid_cells_level;

    map->getOccupiedCellsWithOffset(occupied_grid_cells_level, occupied_cells_level, l);
    occupied_cell_offsets.push_back(occupied_cells_level);
    occupied_grid_cells.push_back(occupied_grid_cells_level);
  }

  for (int l = cell_sizes.size() - 1; l >= 0; l--)
  {
    float cell_size = cell_sizes[l];

    Eigen::Quaternionf q;
    visualization_msgs::Marker marker;
    marker.header.frame_id = map->getFrameId();
    marker.header.stamp = map->getLastUpdateTimestamp();
    marker.ns = "marker_test_occupancy_information";
    marker.id = marker_id++;
    marker.lifetime = ros::Duration(0);
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = cell_size;
    marker.scale.y = cell_size;
    marker.scale.z = cell_size;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.1f;  // std::max( std::min(m_map[ iz ][ iy ][ ix ], 1.f), 0.f);
    marker.pose.orientation.x = 0.f;
    marker.pose.orientation.y = 0.f;
    marker.pose.orientation.z = 0.f;
    marker.pose.orientation.w = 1.f;
    marker.points.resize(occupied_cell_offsets[l].size());
    marker.colors.resize(occupied_cell_offsets[l].size());

    for (size_t i = 0; i < occupied_cell_offsets[l].size(); i++)
    {
      geometry_msgs::Point p;
      p.x = occupied_cell_offsets[l][i](0);  // - cellSize/2.f;
      p.y = occupied_cell_offsets[l][i](1);  // - cellSize/2.f;
      p.z = occupied_cell_offsets[l][i](2);  // - cellSize/2.f;

      std_msgs::ColorRGBA c;

      switch (occupied_grid_cells[l][i].m_debugState)
      {
		case MapType::GridCellType::UNINITIALIZED:
          c.r = 0.7f;
          c.g = 0.7f;
          c.b = 0.7f;
          c.a = 0.1f;
          break;
		case MapType::GridCellType::FREE:
          c.r = 0.f;
          c.g = 1.f;
          c.b = 1.f;
          c.a = 0.f;
          break;

		case MapType::GridCellType::OCCUPIED:
          c.r = 0.f;
          c.g = 1.f;
          c.b = 1.f;
          c.a = 1.f;
          break;

		case MapType::GridCellType::UNKNOWN:
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
    pub_marker_.publish(marker);
  }
}

template <typename PointType, typename MapType>
void MapPublisher::publishMap(const boost::shared_ptr<MapType>& map)
{
  if (pub_map_msg_.getNumSubscribers() == 0)
    return;
  pcl::StopWatch timer;
  timer.reset();

  mrs_laser_mapping::MultiResolutionMapPtr map_msg(new mrs_laser_mapping::MultiResolutionMap);
  map_msg->header.stamp = map->getLastUpdateTimestamp();
  map_msg->header.frame_id = map->getFrameId();
  map_msg->levels = map->getLevels();
  map_msg->size_in_meters = map->getSizeInMeters();
  map_msg->resolution = map->getResolution();
  map_msg->cell_capacity = map->getCellCapacity();

  unsigned int nr_points = 0;
  for (unsigned int i = 0; i < map->getLevels(); i++)
  {
    typename pcl::PointCloud<PointType>::Ptr points(new pcl::PointCloud<PointType>());
    map->getCellPoints(points, i);

    sensor_msgs::PointCloud2 ros_cloud;

    pcl::toROSMsg(*points, ros_cloud);

    map_msg->cloud.push_back(ros_cloud);
    nr_points += points->size();
  }

  /*
   * get all free cells
   */
  std::vector<float> cell_sizes;
  std::vector<std::vector<Eigen::Vector3f>> occupied_cell_offsets;
  typename MapType::CellPointerVectorVector occupied_grid_cells;
  int levels = map->getLevels();

  for (int l = 0; l < levels; l++)
  {
    cell_sizes.push_back(map->getCellSize(l));

    std::vector<Eigen::Vector3f> occupied_cells_level;
    typename MapType::CellPointerVector occupied_grid_cells_level;

    map->getCellsWithOffset(occupied_grid_cells_level, occupied_cells_level, l, -99999.f);  // TODO: this sucks...

    occupied_cell_offsets.push_back(occupied_cells_level);
    occupied_grid_cells.push_back(occupied_grid_cells_level);
  }

  for (int l = cell_sizes.size() - 1; l >= 0; l--)
  {
    for (size_t i = 0; i < occupied_cell_offsets[l].size(); i++)
    {
      if ((*occupied_grid_cells[l][i]).getOccupancy() <= 0.f)
      {  // thos
         // 				if ( occupiedGridCells[l][i]->m_debugState == mrs_laser_maps::GridCellWithStatistics::FREE ) {
        geometry_msgs::Point p;
        p.x = occupied_cell_offsets[l][i](0);
        p.y = occupied_cell_offsets[l][i](1);
        p.z = occupied_cell_offsets[l][i](2);

        map_msg->free_cell_centers.push_back(p);
      }
    }
  }

  pub_map_msg_.publish(map_msg);
  ROS_DEBUG_STREAM_NAMED("map_publisher","published map message took: " << timer.getTime() << " ms with " << nr_points
                                                 << " points. from timestamp " << map_msg->header.stamp << " at time "
                                                 << ros::Time::now());
}

template <typename PointType, typename MapType>
void MapPublisher::publishMapLevelColor(const boost::shared_ptr<MapType>& map)
{
  if (pub_cloud_level_color_.getNumSubscribers() == 0)
    return;

  typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr cellPointsCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  for (size_t l = 0; l < map->getLevels(); ++l)
  {
    double levelNumberNormalized = (static_cast<double>(l + 1) / map->getLevels());

    typename pcl::PointCloud<PointType>::Ptr levelPointsCloud(new pcl::PointCloud<PointType>());
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
  pub_cloud_level_color_.publish(cellPointsCloud);
}


template <typename PointType, typename MapType>
void MapPublisher::publishMapScanColor(const boost::shared_ptr<MapType>& map)
{
  if (pub_cloud_scan_color_.getNumSubscribers() == 0)
    return;

  pcl::PointCloud<PointXYZRGBScanLabel>::Ptr cloud_map(new pcl::PointCloud<PointXYZRGBScanLabel>());
  pcl::StopWatch watch;
  
  map->getCellPoints(cloud_map);

//   ROS_INFO_STREAM("get all cvellpoints took: " << watch.getTime());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pub(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::copyPointCloud(*cloud_map, *cloud_pub);
  
  
  for (size_t i = 0; i < cloud_map->size(); ++i)
  {
    double scan_number_normalized = fmod((static_cast<double>(cloud_map->points[i].scanNr) / 100.0), 1.0);
    float r, g, b;
    ColorMapJet::getRainbowColor(scan_number_normalized, r, g, b);
  
    cloud_pub->points[i].r = static_cast<int>(255.f * r);
    cloud_pub->points[i].g = static_cast<int>(255.f * g);
    cloud_pub->points[i].b = static_cast<int>(255.f * b);
  }
  
  cloud_pub->header.frame_id = map->getFrameId();
  cloud_pub->header.stamp = pcl_conversions::toPCL(map->getLastUpdateTimestamp());
  pub_cloud_scan_color_.publish(cloud_pub);

}


}

#endif
