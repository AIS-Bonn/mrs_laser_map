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
template <typename MapPointType, typename BufferType>
MultiResolutionalMap<MapPointType, BufferType>::MultiResolutionalMap(int size, float resolution, int levels, int cellCapacity,
                                                   std::string frameId)
  : size_(size)
  , resolution_(resolution)
  , frame_id_(frameId)
  , cell_capacity_(cellCapacity)
  , last_update_stamp_(0)
  , scan_number_(0)
  , evaluated_(false)
{
  initMap();

  for (int i = 0; i < levels; i++)
  {
    float levelSize = size / pow(2, (i));
    float cellsPerMeter = 1 / (levelSize / (2.f * resolution));
    MapLevelPtr map_ptr(boost::make_shared<MapLevelType>(levelSize, cellsPerMeter, cellCapacity));
    level_maps_.insert(level_maps_.begin(), map_ptr);
    ROS_DEBUG_NAMED("map", "creating map layer with %f length and %f resolution", levelSize, map_ptr->getResolution());
  }

  for (int i = 0; i < levels - 1; i++)
  {
    level_maps_[i]->setCoarserLevel(level_maps_[i + 1]);
  }
}

template <typename MapPointType, typename BufferType>
MultiResolutionalMap<MapPointType, BufferType>::MultiResolutionalMap(const MultiResolutionalMap& map)
   : size_(map.getSizeInMeters())
   , resolution_(map.getResolution())
   , frame_id_(map.getFrameId())
   , cell_capacity_(map.getCellCapacity())
   , last_update_stamp_(map.getLastUpdateTimestamp())
   , scan_number_(map.getScanNumber())
   , evaluated_(map.isEvaluated())
{
  for ( auto level: map.level_maps_)
    level_maps_.push_back(boost::make_shared<MapLevel<MapPointType>>(*level.get())); 

  for (int i = 1; i < level_maps_.size() ; i++)
  {
    level_maps_[i-1]->setCoarserLevel(level_maps_[i]);
  }
  ROS_DEBUG_NAMED("map", "MultiResolutionalMap<MapPointType, BufferType>::MultiResolutionalMap(const MultiResolutionalMap& map)");
}

template <typename MapPointType, typename BufferType>
void MultiResolutionalMap<MapPointType, BufferType>::initMap()
{
}

template <typename MapPointType, typename BufferType>
void MultiResolutionalMap<MapPointType, BufferType>::translateMap(const Eigen::Vector3f& translation)
{
  for (LevelListReverseIterator it = level_maps_.rbegin(); it != level_maps_.rend(); ++it)
  {
    (*it)->translateMap(translation);
  }
  
  evaluated_ = false;
}

template <typename MapPointType, typename BufferType>
void MultiResolutionalMap<MapPointType, BufferType>::set(const MapPointType& p, bool update_occupancy)
{
  for (auto& level : boost::adaptors::reverse(level_maps_) )
  {
    // if not added in this level (point not in map level) we can skip the finer levels 
    if (!level->set(p, update_occupancy))
      break;
  }
  evaluated_ = false;
}

template <typename MapPointType, typename BufferType>
void MultiResolutionalMap<MapPointType, BufferType>::getOccupiedCells(std::vector<pcl::PointXYZ>& cells )
{
  cells.clear();
  for (const MapLevelPtr& level : level_maps_)
  {
    level->getOccupiedCells(cells );
  }
}

template <typename MapPointType, typename BufferType>
void MultiResolutionalMap<MapPointType, BufferType>::getOccupiedCells(std::vector<pcl::PointXYZ>& cells, int level)
{
  cells.clear();

  if (level >= 0 && level < static_cast<int>(level_maps_.size()))
  {
    level_maps_[level]->getOccupiedCells(cells, (level > 0) );
  }
}

template <typename MapPointType, typename BufferType>
void MultiResolutionalMap<MapPointType, BufferType>::getOccupiedCells(AlignedCellVector& cells, int level)
{
  cells.clear();

  std::vector<Eigen::Vector3f> offsets;

  if (level >= 0 && level < static_cast<int>(level_maps_.size()))
  {
    level_maps_[level]->getOccupiedCellsWithOffset(cells, offsets, (level > 0));
  }
}

template <typename MapPointType, typename BufferType>
void MultiResolutionalMap<MapPointType, BufferType>::getOccupiedCellsWithOffset(mrs_laser_maps::SurfelMapInterface::CellPtrVector& cells, std::vector<Eigen::Vector3f>& offsets,
                                                      int level)
{
  cells.clear();

  if (level >= 0 && level < static_cast<int>(level_maps_.size()))
  {
    level_maps_[level]->getCellsWithOffset(cells, offsets, (level > 0), OCCUPANCY_UNKNOWN);
  }
}

template <typename MapPointType, typename BufferType>
void MultiResolutionalMap<MapPointType, BufferType>::getCellsWithOffset(CellPointerVector& cells, std::vector<Eigen::Vector3f>& offsets,
                                                      int level, float occupancyThreshold)
{
  cells.clear();

  if (level >= 0 && level < static_cast<int>(level_maps_.size()))
  {
    level_maps_[level]->getCellsWithOffset(cells, offsets, (level > 0), occupancyThreshold);
  }
}

template <typename MapPointType, typename BufferType>
void MultiResolutionalMap<MapPointType, BufferType>::getOccupiedCellsWithOffset(AlignedCellVector& cells,
                                                              std::vector<Eigen::Vector3f>& offsets, int level)
{
  cells.clear();

  if (level >= 0 && level < static_cast<int>(level_maps_.size()))
  {
    level_maps_[level]->getOccupiedCellsWithOffset(cells, offsets, (level > 0));
  }
}


template <typename MapPointType, typename BufferType>
void MultiResolutionalMap<MapPointType, BufferType>::getOccupiedCellsWithOffset(CellPointerVector& cells,
                                                              std::vector<Eigen::Vector3f>& offsets)
{
  cells.clear();
  offsets.clear();

  for (size_t i = 0; i < level_maps_.size(); i++)
  {
    level_maps_[i]->getOccupiedCellsWithOffset(cells, offsets, (i > 0));
  }
}

template <typename MapPointType, typename BufferType>
void MultiResolutionalMap<MapPointType, BufferType>::getOccupiedCellsWithOffset(mrs_laser_maps::SurfelMapInterface::CellPtrVector& cells,
                                                              std::vector<Eigen::Vector3f>& offsets)
{
  cells.clear();
  offsets.clear();

  for (size_t i = 0; i < level_maps_.size(); i++)
  {
    level_maps_[i]->getOccupiedCellsWithOffset(cells, offsets, (i > 0));
  }
}

template <typename MapPointType, typename BufferType>
void MultiResolutionalMap<MapPointType, BufferType>::getCellPoints(PointCloudPtr points)
{
  points->clear();
  for (size_t i = 0; i < level_maps_.size(); i++)
  {
    level_maps_[i]->getCellPoints(points, (i > 0));
  }
}

template <typename MapPointType, typename BufferType>
void MultiResolutionalMap<MapPointType, BufferType>::getCellPointsDownsampled(PointCloudPtr points, unsigned int pointsPerCell)
{
  points->clear();

  for (LevelListIterator it = level_maps_.begin(); it != level_maps_.end(); ++it)
  {
    (*it)->getCellPointsDownsampled(points, pointsPerCell);
  }
}

template <typename MapPointType, typename BufferType>
void MultiResolutionalMap<MapPointType, BufferType>::getCellPoints(PointCloudPtr points, int level)
{
  if (level >= 0 && level < static_cast<int>(level_maps_.size()))
  {
    level_maps_[level]->getCellPoints(points);
  }
}

template <typename MapPointType, typename BufferType>
unsigned int MultiResolutionalMap<MapPointType, BufferType>::getNumCellPoints()
{
  unsigned int num = 0;
  for (size_t i = 0; i < level_maps_.size(); i++)
  {
    num += level_maps_[i]->getNumCellPoints();
  }
// ROS_INFO_STREAM("getNumCellPoints: " << num); 
  return num;
}

template <typename MapPointType, typename BufferType>
bool MultiResolutionalMap<MapPointType, BufferType>::getCell(const Eigen::Vector3f& point, mrs_laser_maps::SurfelMapInterface::CellPtrVector& cellPtrs,
                      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& cellOffsets,
                      std::vector<int>& levels, int neighbors, bool reverse)
  {
    if (reverse)
    {
      for (int i = level_maps_.size() - 1; i >= 0; --i)
      {
        if (level_maps_[i]->getCell(point, cellPtrs, cellOffsets, levels, i, neighbors))
        {
          return true;
        }
      }
    }
    else
    {
      for (unsigned int i = 0; i < level_maps_.size(); i++)
      {
        if (level_maps_[i]->getCell(point, cellPtrs, cellOffsets, levels, i, neighbors))
        {
          return true;
        }
      }
    }
    return false;
  }

template <typename MapPointType, typename BufferType>
void MultiResolutionalMap<MapPointType, BufferType>::addCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool updateOccupancy, const Eigen::Matrix4f& origin)
{
  PointCloudPtr cloud_map_type(new PointCloud);
  pcl::copyPointCloud(*cloud, *cloud_map_type);
  addCloudInner(cloud_map_type, updateOccupancy, origin);
}

template <typename MapPointType, typename BufferType>
void MultiResolutionalMap<MapPointType, BufferType>::addCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, bool updateOccupancy, const Eigen::Matrix4f& origin)
{
  PointCloudPtr cloud_map_type(new PointCloud);
  pcl::copyPointCloud(*cloud, *cloud_map_type);
  addCloudInner(cloud_map_type, updateOccupancy, origin);
}

template <typename MapPointType, typename BufferType>
void MultiResolutionalMap<MapPointType, BufferType>::addCloud(pcl::PointCloud<PointXYZScanLine>::Ptr cloud, bool updateOccupancy, const Eigen::Matrix4f& origin)
{
  PointCloudPtr cloud_map_type(new PointCloud);
  pcl::copyPointCloud(*cloud, *cloud_map_type);
  addCloudInner(cloud_map_type, updateOccupancy, origin);
}

template <typename MapPointType, typename BufferType>
void MultiResolutionalMap<MapPointType, BufferType>::addCloud(pcl::PointCloud<PointXYZRGBScanLabel>::Ptr cloud, bool updateOccupancy, const Eigen::Matrix4f& origin)
{
  PointCloudPtr cloud_map_type(new PointCloud);
  pcl::copyPointCloud(*cloud, *cloud_map_type);
  addCloudInner(cloud_map_type, updateOccupancy, origin);
}

// template <>
// void MultiResolutionalMap<PointXYZRGBScanLabel, mrs_laser_maps::cell_buffer<PointXYZRGBScanLabel>>::setScanNumber(pcl::PointCloud<PointXYZRGBScanLabel>::Ptr cloud)
// {
//   for (size_t i = 0; i < cloud->size(); ++i)
//   {
//     cloud->points[i].scanNr = scan_number_;
//   }
//   scan_number_++;
// }

template <>
void MultiResolutionalMap<PointXYZRGBScanLabel, boost::circular_buffer_space_optimized<PointXYZRGBScanLabel>>::setScanNumber(pcl::PointCloud<PointXYZRGBScanLabel>::Ptr cloud)
{
  for (auto& point : cloud->points)
  {
    point.scanNr = scan_number_;
  }
  scan_number_++;
}

template <typename MapPointType, typename BufferType>
void MultiResolutionalMap<MapPointType, BufferType>::addCloudInner(PointCloudPtr cloud, bool update_occupancy, const Eigen::Matrix4f& origin)
{
  ROS_DEBUG_STREAM_NAMED("map", "addCloudInner with scan_number_: " << scan_number_);
  
  boost::mutex::scoped_lock lock(mutex_);

  setScanNumber(cloud);
  
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

#ifdef DEBUG_CELL_HITS  
  for (MapLevelPtr l : level_maps_)
  {
    l->printCellDebugInfo();
  }
#endif
  
  evaluated_ = false;
  ros::Time last_update_time;
  setLastUpdateTimestamp(last_update_time.fromNSec(cloud->header.stamp));
}

template <typename MapPointType, typename BufferType>
void MultiResolutionalMap<MapPointType, BufferType>::setUpdateMask(std::vector<std::vector<std::vector<bool>>>& updateMask)
{
  for (LevelListIterator it = level_maps_.begin(); it != level_maps_.end(); ++it)
  {
    (*it)->setUpdateMask(updateMask);
  }
}

template <typename MapPointType, typename BufferType>
void MultiResolutionalMap<MapPointType, BufferType>::setConicalUpdateMask(const Eigen::Vector3f& position,
                                                        const Eigen::Vector3f& orientation, float angle)
{
  for (LevelListIterator it = level_maps_.begin(); it != level_maps_.end(); ++it)
  {
    (*it)->setConicalUpdateMask(position, orientation, 0.78);
  }
}

template <typename MapPointType, typename BufferType>
void MultiResolutionalMap<MapPointType, BufferType>::setOccupancyParameters(float clampingThreshMin, float clampingThreshMax,
                                                          float probHit, float probMiss)
{
  for (LevelListIterator it = level_maps_.begin(); it != level_maps_.end(); ++it)
  {
    (*it)->setOccupancyParameters(clampingThreshMin, clampingThreshMax, probHit, probMiss);
  }
}

template <typename MapPointType, typename BufferType>
void MultiResolutionalMap<MapPointType, BufferType>::evaluateAll()
{
  if (evaluated_ == true)
  {
    ROS_INFO("calling evaluate all but map is already evaluated");
  }
  
  for (const MapLevelPtr& level : level_maps_)
  {
    level->evaluateAll();
  }
  evaluated_ = true;
}

template <typename MapPointType, typename BufferType>
void MultiResolutionalMap<MapPointType, BufferType>::unEvaluateAll()
{
  for (const MapLevelPtr& level : level_maps_)
  {
    level->unEvaluateAll();
  }
  evaluated_ = false;
}

}

#endif
