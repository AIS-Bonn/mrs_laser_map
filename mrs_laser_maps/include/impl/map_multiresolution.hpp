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

#ifndef _MAP_MULTIRESOLUTION_IMPL_H_
#define _MAP_MULTIRESOLUTION_IMPL_H_

#include <mrs_laser_maps/map_multiresolution.h>

namespace mrs_laser_maps
{
template <typename MapPointType>
MultiResolutionalMap<MapPointType>::MultiResolutionalMap(int size, float resolution, int levels, int cellCapacity,
                                                   std::string frameId)
  : size_(size)
  , resolution_(resolution)
  , frame_id_(frameId)
  , cell_capacity_(cellCapacity)
  , last_update_stamp_(0)
{
  initMap();

  for (int i = 0; i < levels; i++)
  {
    float levelSize = size / pow(2, (i));
    float cellsPerMeter = 1 / (levelSize / (2.f * resolution));
    MapLevelPtr map_ptr(new MapLevel<MapPointType>(levelSize, cellsPerMeter, cellCapacity));
    level_maps_.insert(level_maps_.begin(), map_ptr);
    ROS_DEBUG_NAMED("map", "creating map layer with %f length and %f resolution", levelSize, map_ptr->getResolution());
  }

  for (int i = 0; i < levels - 1; i++)
  {
    level_maps_[i]->setCoarserLevel(level_maps_[i + 1]);
  }
}

template <typename MapPointType>
MultiResolutionalMap<MapPointType>::MultiResolutionalMap(MultiResolutionalMap& map)
  : size_(map.getSizeInMeters())
  , resolution_(map.getResolution())
  , frame_id_(map.getFrameId())
  , cell_capacity_(map.getCellCapacity())
  , last_update_stamp_(0)
{
  initMap();
  int levels = map.getLevels();

  for (int i = 0; i < levels; i++)
  {
    float levelSize = size_ / pow(2, (i));
    float cellsPerMeter = 1 / (levelSize / (2.f * resolution_));
    MapLevelPtr map_ptr(new MapLevel<MapPointType>(levelSize, cellsPerMeter, cell_capacity_));
    level_maps_.insert(level_maps_.begin(), map_ptr);
    ROS_DEBUG("creating map layer with %f length and %f resolution", levelSize, map_ptr->getResolution());
  }

  for (unsigned int i = 0; i < level_maps_.size(); i++)
  {
    PointCloudPtr points(new pcl::PointCloud<MapPointType>());
    map.getCellPoints(points, i);
    for (size_t j = 0; j < points->size(); j++)
    {
      level_maps_[i]->set(points->points[j]);
    }
  }

  for (int i = 0; i < levels - 1; i++)
  {
    level_maps_[i]->setCoarserLevel(level_maps_[i + 1]);
  }
}

template <typename MapPointType>
void MultiResolutionalMap<MapPointType>::initMap()
{
}

template <typename MapPointType>
void MultiResolutionalMap<MapPointType>::moveMap(const Eigen::Vector3f& translation)
{
  for (unsigned int i = 0; i < level_maps_.size(); i++)
  {
    level_maps_[i]->moveMap(translation);
  }
}

template <typename MapPointType>
void MultiResolutionalMap<MapPointType>::translateMap(const Eigen::Vector3f& translation)
{
  for (LevelListReverseIterator it = level_maps_.rbegin(); it != level_maps_.rend(); ++it)
  {
    (*it)->translateMap(translation);
  }
}

template <typename MapPointType>
void MultiResolutionalMap<MapPointType>::getOccupiedCells(std::vector<pcl::PointXYZ>& cells )
{
  cells.clear();

  for (size_t i = 0; i < level_maps_.size(); i++)
  {
    level_maps_[i]->getOccupiedCells(cells );
  }
}

template <typename MapPointType>
void MultiResolutionalMap<MapPointType>::getOccupiedCells(std::vector<pcl::PointXYZ>& cells, int level)
{
  cells.clear();

  if (level >= 0 && level < static_cast<int>(level_maps_.size()))
  {
    level_maps_[level]->getOccupiedCells(cells, (level > 0) );
  }
}

template <typename MapPointType>
void MultiResolutionalMap<MapPointType>::getOccupiedCells(AlignedCellVector& cells, int level)
{
  cells.clear();

  std::vector<Eigen::Vector3f> offsets;

  if (level >= 0 && level < static_cast<int>(level_maps_.size()))
  {
    level_maps_[level]->getOccupiedCellsWithOffset(cells, offsets, (level > 0));
  }
}

template <typename MapPointType>
void MultiResolutionalMap<MapPointType>::getOccupiedCellsWithOffset(CellPointerVector& cells,
                                                              std::vector<Eigen::Vector3f>& offsets, int level)
{
  getCellsWithOffset(cells, offsets, level, OCCUPANCY_UNKNOWN);
}

template <typename MapPointType>
void MultiResolutionalMap<MapPointType>::getCellsWithOffset(CellPointerVector& cells, std::vector<Eigen::Vector3f>& offsets,
                                                      int level, float occupancyThreshold)
{
  cells.clear();

  if (level >= 0 && level < static_cast<int>(level_maps_.size()))
  {
    level_maps_[level]->getCellsWithOffset(cells, offsets, (level > 0), occupancyThreshold);
  }
}

template <typename MapPointType>
void MultiResolutionalMap<MapPointType>::getOccupiedCellsWithOffset(AlignedCellVector& cells,
                                                              std::vector<Eigen::Vector3f>& offsets, int level)
{
  // 	mutex_.lock();

  cells.clear();

  if (level >= 0 && level < static_cast<int>(level_maps_.size()))
  {
    level_maps_[level]->getOccupiedCellsWithOffset(cells, offsets, (level > 0));
  }

  // 	mutex_.unlock();
}

template <typename MapPointType>
void MultiResolutionalMap<MapPointType>::getOccupiedCellsWithOffset(CellPointerVector& cells,
                                                              std::vector<Eigen::Vector3f>& offsets)
{
  // 	mutex_.lock();

  cells.clear();
  offsets.clear();

  for (size_t i = 0; i < level_maps_.size(); i++)
  {
    // level_maps_[i]->getOccupiedCellsWithOffset( cells, offsets,  (i> 0) );
    level_maps_[i]->getOccupiedCellsWithOffset(cells, offsets, (i > 0));
  }

  // 	mutex_.unlock();
}

template <typename MapPointType>
void MultiResolutionalMap<MapPointType>::getCellPoints(PointCloudPtr points)
{
  // 	mutex_.lock();
  points->clear();
  for (size_t i = 0; i < level_maps_.size(); i++)
  {
    level_maps_[i]->getCellPoints(points, (i > 0));
    // level_maps_[i]->getCellPoints( points, false );
  }

  // 	mutex_.unlock();
}

template <typename MapPointType>
void MultiResolutionalMap<MapPointType>::getCellPointsDownsampled(PointCloudPtr points, unsigned int pointsPerCell)
{
  // 	mutex_.lock();
  points->clear();

  for (LevelListIterator it = level_maps_.begin(); it != level_maps_.end(); ++it)
  {
    (*it)->getCellPointsDownsampled(points, pointsPerCell);
  }
  // 	mutex_.unlock();
}

template <typename MapPointType>
void MultiResolutionalMap<MapPointType>::getCellPoints(PointCloudPtr points, int level)
{
  // 	mutex_.lock();

  if (level >= 0 && level < static_cast<int>(level_maps_.size()))
  {
    level_maps_[level]->getCellPoints(points);
  }

  // 	mutex_.unlock();
}

template <typename MapPointType>
unsigned int MultiResolutionalMap<MapPointType>::getNumCellPoints()
{
  unsigned int num = 0;
  for (size_t i = 0; i < level_maps_.size(); i++)
  {
    num += level_maps_[i]->getNumCellPoints();
    //		ROS_INFO("points in level %d %d", i, level_maps_[i]->getNumCellPoints());
  }

  return num;
}

// template <>
// void MultiResolutionalMap<PointXYZRGBScanLabel>::addCloud(pcl::PointCloud<PointXYZScanLine>::Ptr cloud, bool updateOccupancy, const Eigen::Matrix4f& origin)
// {
//  	
// // 	pcl::PointCloud<mrs_laser_maps::MapMapPointTypeype>::Ptr cloud_map_type(new pcl::PointCloud<mrs_laser_maps::MapMapPointTypeype>); // TODO
// //   
// //   pcl::copyPointCloud(*cloud_map_type, *cloud_map_type);
// //   double scanNumberNormalized = fmod((static_cast<double>(scan_number_) / 25.0), 1.0);
// //   for (size_t i = 0; i < cloud_map_type->size(); ++i)
// //   {
// //     cloud_map_type->points[i].r = static_cast<int>(255.f * ColorMapJet::red(scanNumberNormalized));
// //     cloud_map_type->points[i].g = static_cast<int>(255.f * ColorMapJet::green(scanNumberNormalized));
// //     cloud_map_type->points[i].b = static_cast<int>(255.f * ColorMapJet::blue(scanNumberNormalized));
// //     cloud_map_type->points[i].scanNr = scan_number_;
// //   }
// 
// }

template <typename MapPointType>
void MultiResolutionalMap<MapPointType>::addCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool updateOccupancy, const Eigen::Matrix4f& origin)
{
	PointCloudPtr cloud_map_type(new PointCloud);
	pcl::copyPointCloud(*cloud, *cloud_map_type);
	addCloudInner(cloud_map_type, updateOccupancy, origin);
}

template <typename MapPointType>
void MultiResolutionalMap<MapPointType>::addCloud(pcl::PointCloud<PointXYZScanLine>::Ptr cloud, bool updateOccupancy, const Eigen::Matrix4f& origin)
{
 	PointCloudPtr cloud_map_type(new PointCloud);
	pcl::copyPointCloud(*cloud, *cloud_map_type);
	addCloudInner(cloud_map_type, updateOccupancy, origin);
}

template <typename MapPointType>
void MultiResolutionalMap<MapPointType>::addCloud(pcl::PointCloud<PointXYZRGBScanLabel>::Ptr cloud, bool updateOccupancy, const Eigen::Matrix4f& origin)
{
 	PointCloudPtr cloud_map_type(new PointCloud);
	pcl::copyPointCloud(*cloud, *cloud_map_type);
	addCloudInner(cloud_map_type, updateOccupancy, origin);
}

template <typename MapPointType>
void MultiResolutionalMap<MapPointType>::addCloudInner(PointCloudPtr cloud, bool update_occupancy, const Eigen::Matrix4f& origin)
{
  boost::mutex::scoped_lock lock(mutex_);

  // unmark all endpoints
  setAllEndPointFlags(false);
  
  // mark endpoints
  if (update_occupancy)
  {
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
      setEndPointFlag(cloud->points[i], origin);
    }
  }

  for (size_t i = 0; i < cloud->points.size(); i++)
  {
    if (update_occupancy)
      insertRay(cloud->points[i], origin);
    else
      set(cloud->points[i]);
  }

  ros::Time last_update_time;
  setLastUpdateTimestamp(last_update_time.fromNSec(cloud->header.stamp));
}

template <typename MapPointType>
pcl::PointXYZ MultiResolutionalMap<MapPointType>::getOdomentryIncrement(int level)
{
  if (level >= 0 && level < (int)level_maps_.size())
  {
    return level_maps_[level]->getOdomentryIncrement();
  }
  return pcl::PointXYZ();
}

template <typename MapPointType>
void MultiResolutionalMap<MapPointType>::setUpdateMask(std::vector<std::vector<std::vector<bool>>>& updateMask)
{
  for (LevelListIterator it = level_maps_.begin(); it != level_maps_.end(); ++it)
  {
    (*it)->setUpdateMask(updateMask);
  }
}

template <typename MapPointType>
void MultiResolutionalMap<MapPointType>::setConicalUpdateMask(const Eigen::Vector3f& position,
                                                        const Eigen::Vector3f& orientation, float angle)
{
  for (LevelListIterator it = level_maps_.begin(); it != level_maps_.end(); ++it)
  {
    (*it)->setConicalUpdateMask(position, orientation, 0.78);
  }
}

template <typename MapPointType>
void MultiResolutionalMap<MapPointType>::setOccupancyParameters(float clampingThreshMin, float clampingThreshMax,
                                                          float probHit, float probMiss)
{
  for (LevelListIterator it = level_maps_.begin(); it != level_maps_.end(); ++it)
  {
    (*it)->setOccupancyParameters(clampingThreshMin, clampingThreshMax, probHit, probMiss);
  }
}
}

#endif
