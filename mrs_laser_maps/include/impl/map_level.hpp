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

#ifndef _MAP_LEVEL_IMPL_H_
#define _MAP_LEVEL_IMPL_H_

#include <mrs_laser_maps/map_level.h>

namespace mrs_laser_maps
{
template <typename PointT>
MapLevel<PointT>::MapLevel(int size_in_meters, float resolution, int cell_capacity)
  : size_(size_in_meters)
  , resolution_(resolution)
  , cell_capacity_(cell_capacity)
  , translation_increment_(0.f, 0.f, 0.f)
  , update_mask_(size_ * resolution_,
                 std::vector<std::vector<bool>>(size_ * resolution_,
                                                std::vector<bool>(size_ * resolution_, true)))
  , clamping_thresh_min_(-2.f)
  , clamping_thresh_max_(3.5f)
  , prob_hit_(0.85f)
  , prob_miss_(-0.4f)
{
  initMap();
}

template <typename PointT>
MapLevel<PointT>::~MapLevel()
{
}

template <typename PointT>
void MapLevel<PointT>::initMap()
{
  map_.resize(size_ * resolution_);

  for (int i = 0; i < size_ * resolution_; i++)
    map_.push_back(boost::circular_buffer<boost::circular_buffer<GridCellType>>(
        size_ * resolution_,
        boost::circular_buffer<GridCellType>(size_ * resolution_, GridCellType(cell_capacity_))));

  for (size_t iz = 0; iz < size_ * resolution_; iz++)
  {
    for (size_t iy = 0; iy < size_ * resolution_; iy++)
    {
      for (size_t ix = 0; ix < size_ * resolution_; ix++)
      {
        map_[iz][iy][ix] = GridCellType(cell_capacity_);
      }
    }
  }

  for (size_t iz = 0; iz < size_ * resolution_; iz++)
  {
    for (size_t iy = 0; iy < size_ * resolution_; iy++)
    {
      for (size_t ix = 0; ix < size_ * resolution_; ix++)
      {
        update_mask_[iz][iy][ix] = true;
      }
    }
  }

  setAllOccupancies(OCCUPANCY_UNKNOWN);

  ROS_DEBUG_STREAM("map with " << map_.size() << " m_resolu " << resolution_);
}

template <typename PointT>
void MapLevel<PointT>::moveMap(const Eigen::Vector3f& translation)
{
  int mx = translation(0) * resolution_;
  int my = translation(1) * resolution_;
  int mz = translation(2) * resolution_;

  //  	ROS_DEBUG_NAMED("map", "moving map %d %d %d with %d points", mx, my, mz, getNumCellPoints());
  ROS_DEBUG_NAMED("map", "moving map %d %d %d", mx, my, mz);

  // move map in z direction
  for (int iz = 0; iz < mz; iz++)
  {
    map_.push_back(boost::circular_buffer<boost::circular_buffer<GridCellType>>(
        size_ * resolution_,
        boost::circular_buffer<GridCellType>(size_ * resolution_, GridCellType(cell_capacity_))));
    for (size_t iy = 0; iy < map_.size(); iy++)
    {
      map_[map_.size() - 1].push_back(
          boost::circular_buffer<GridCellType>(size_ * resolution_, GridCellType(cell_capacity_)));

      for (size_t ix = 0; ix < map_.size(); ix++)
      {
        map_[map_.size() - 1][map_.size() - 1].push_back(GridCellType(cell_capacity_));
        map_[map_.size() - 1][map_.size() - 1][map_.size() - 1].getPoints()->clear();
      }
    }
  }
  size_t ic = map_.size() - 1;
  for (int iz = 0; iz < mz; iz++, ic--)
    for (size_t iy = 0; iy < map_.size(); iy++)
      for (size_t ix = 0; ix < map_.size(); ix++)
        retainPoints(ic, iy, ix);

  for (int iz = mz; iz < 0; iz++)
  {
    map_.push_front(boost::circular_buffer<boost::circular_buffer<GridCellType>>(
        size_ * resolution_,
        boost::circular_buffer<GridCellType>(size_ * resolution_, GridCellType(cell_capacity_))));
    for (size_t iy = 0; iy < map_.size(); iy++)
    {
      map_[0].push_back(
          boost::circular_buffer<GridCellType>(size_ * resolution_, GridCellType(cell_capacity_)));
      for (size_t ix = 0; ix < map_.size(); ix++)
      {
        map_[0][map_.size() - 1].push_back(GridCellType(cell_capacity_));
        map_[0][map_.size() - 1][map_.size() - 1].getPoints()->clear();
        // 				retainPoints( 0, map_.size()-1, map_.size()-1);
      }
    }
  }
  ic = 0;
  for (int iz = mz; iz < 0; iz++, ic++)
    for (size_t iy = 0; iy < map_.size(); iy++)
      for (size_t ix = 0; ix < map_.size(); ix++)
        retainPoints(ic, iy, ix);

  // move map in y direction
  for (size_t iz = 0; iz < map_.size(); iz++)
  {
    for (int iy = 0; iy < my; iy++)
    {
      map_[iz].push_back(
          boost::circular_buffer<GridCellType>(size_ * resolution_, GridCellType(cell_capacity_)));
      for (size_t ix = 0; ix < map_.size(); ix++)
      {
        map_[iz][map_.size() - 1].push_back(GridCellType(cell_capacity_));
        map_[iz][map_.size() - 1][map_.size() - 1].getPoints()->clear();
        // 				retainPoints( iz, map_.size()-1 , map_.size()-1);
      }
    }
    for (int iy = my; iy < 0; iy++)
    {
      map_[iz].push_front(
          boost::circular_buffer<GridCellType>(size_ * resolution_, GridCellType(cell_capacity_)));
      for (size_t ix = 0; ix < map_.size(); ix++)
      {
        map_[iz][0].push_back(GridCellType(cell_capacity_));
        map_[iz][0][map_.size() - 1].getPoints()->clear();
        // 				retainPoints( iz, 0 , map_.size()-1);
      }
    }
  }
  ic = map_.size() - 1;
  for (int iy = 0; iy < my; iy++, ic--)
    for (size_t iz = 0; iz < map_.size(); iz++)
      for (size_t ix = 0; ix < map_.size(); ix++)
        retainPoints(iz, ic, ix);

  ic = 0;
  for (int iy = my; iy < 0; iy++, ic++)
    for (size_t iz = 0; iz < map_.size(); iz++)
      for (size_t ix = 0; ix < map_.size(); ix++)
        retainPoints(iz, ic, ix);

  // move map in x direction
  for (size_t iz = 0; iz < map_.size(); iz++)
  {
    for (size_t iy = 0; iy < map_[iz].size(); iy++)
    {
      for (int ix = 0; ix < mx; ix++)
      {
        map_[iz][iy].push_back(GridCellType(cell_capacity_));
        map_[iz][iy][map_.size() - 1].getPoints()->clear();
        // 				retainPoints( iz, iy, map_.size()-1 );
      }
      for (int ix = mx; ix < 0; ix++)
      {
        assert(map_[iz][iy].size() == map_.size());
        map_[iz][iy].push_front(GridCellType(cell_capacity_));
        map_[iz][iy][0].getPoints()->clear();
        assert(map_[iz][iy].size() == map_.size());
        // 				retainPoints( iz, iy, 0 );
      }
    }
  }

  ic = map_.size() - 1;
  for (int ix = 0; ix < mx; ix++, ic--)
    for (size_t iz = 0; iz < map_.size(); iz++)
      for (size_t iy = 0; iy < map_.size(); iy++)
        retainPoints(iz, iy, ic);

  ic = 0;
  for (int ix = mx; ix < 0; ix++, ic++)
    for (size_t iz = 0; iz < map_.size(); iz++)
      for (size_t iy = 0; iy < map_.size(); iy++)
        retainPoints(iz, iy, ic);
}

template <typename PointT>
bool MapLevel<PointT>::calcIndices(const PointT& p, int& x, int& y, int& z) const
{
  return calcIndices(p.getVector3fMap(), x, y, z);
}

template <typename PointT>
bool MapLevel<PointT>::calcIndices(const Eigen::Vector3f& p, int& x, int& y, int& z) const
{
  int mid_point = (size_ * resolution_ / 2);
  int max_point = (size_ * resolution_);

  float fx = resolution_ * p(0) + mid_point;
  float fy = resolution_ * p(1) + mid_point;
  float fz = resolution_ * p(2) + mid_point;

  x = resolution_ * p(0) + mid_point;
  y = resolution_ * p(1) + mid_point;
  z = resolution_ * p(2) + mid_point;

  bool valid = fx >= 0.f && fy >= 0.f && fz >= 0.f;
  valid = valid && x >= 0 && y >= 0 && z >= 0;
  valid = valid && fx < (float)max_point && fy < (float)max_point && fz < (float)max_point;

  return valid;
}

template <typename PointT>
void MapLevel<PointT>::translateMap(const Eigen::Vector3f& translation)
{
  if (isnan(translation(0)) || isnan(translation(1)) || isnan(translation(2)))
    return;

  translation_increment_.x += translation(0);
  translation_increment_.y += translation(1);
  translation_increment_.z += translation(2);

  ROS_DEBUG_NAMED("map", "map displace x: %f", translation_increment_.x);
  ROS_DEBUG_NAMED("map", "map displace y: %f", translation_increment_.y);
  ROS_DEBUG_NAMED("map", "map displace z: %f", translation_increment_.z);

  float cell_size = 1.f / resolution_;

  float displacement;
  if (fabsf(translation_increment_.x) >= cell_size)
  {
    Eigen::Vector3f map_displacement = Eigen::Vector3f::Zero();
    displacement = (float)round((int)(translation_increment_.x / cell_size)) * cell_size;
    map_displacement(0) = displacement;
    translation_increment_.x -= (float)displacement;
    moveMap(map_displacement);
  }

  if (fabsf(translation_increment_.y) >= cell_size)
  {
    Eigen::Vector3f map_displacement = Eigen::Vector3f::Zero();
    displacement = (float)round((int)(translation_increment_.y / cell_size)) * cell_size;
    map_displacement(1) = displacement;
    translation_increment_.y -= (float)displacement;
    moveMap(map_displacement);
  }

  if (fabsf(translation_increment_.z) >= cell_size)
  {
    Eigen::Vector3f map_displacement = Eigen::Vector3f::Zero();
    displacement = (float)round((int)(translation_increment_.z / cell_size)) * cell_size;
    map_displacement(2) = displacement;
    translation_increment_.z -= (float)displacement;
    moveMap(map_displacement);
  }
}

template <typename PointT>
void MapLevel<PointT>::decreaseAll(float decreaseRate)
{
  ROS_DEBUG_NAMED("map","MapLevel: decreaseAll");
  for (size_t iz = 0; iz < map_.size(); iz++)
  {
    for (size_t iy = 0; iy < map_[iz].size(); iy++)
    {
      for (size_t ix = 0; ix < map_[iz][iy].size(); ix++)
      {
        if (update_mask_[iz][iy][ix])
        {
          map_[iz][iy][ix].substractOccupancy(decreaseRate);
        }
      }
    }
  }
}

template <typename PointT>
void MapLevel<PointT>::evaluateAll()
{
  for (size_t iz = 0; iz < map_.size(); iz++)
  {
    for (size_t iy = 0; iy < map_[iz].size(); iy++)
    {
      for (size_t ix = 0; ix < map_[iz][iy].size(); ix++)
      {
        map_[iz][iy][ix].surfel_.clear();
        map_[iz][iy][ix].evaluate();
      }
    }
  }
}




template <typename PointT>
void MapLevel<PointT>::unEvaluateAll()
{
  for (size_t iz = 0; iz < map_.size(); iz++)
  {
    for (size_t iy = 0; iy < map_[iz].size(); iy++)
    {
      for (size_t ix = 0; ix < map_[iz][iy].size(); ix++)
        map_[iz][iy][ix].surfel_.unevaluate();
    }
  }
}

template <typename PointT>
inline void MapLevel<PointT>::setAllOccupancies(float occupancy)
{
  ROS_DEBUG("MapLevel<PointT>::setAllOccupancies ");
  for (size_t iz = 0; iz < map_.size(); iz++)
  {
    for (size_t iy = 0; iy < map_[iz].size(); iy++)
    {
      for (size_t ix = 0; ix < map_[iz][iy].size(); ix++)
        map_[iz][iy][ix].setOccupancy(occupancy);
    }
  }
}

template <typename PointT>
int MapLevel<PointT>::getLocalCellPoints(const pcl::PointXYZ& point, CircularBufferIterator& begin,
                                         CircularBufferIterator& end, float& x_off, float& y_off, float& z_off)
{
  int ix, iy, iz;
  if (calcIndices(point, ix, iy, iz))
  {
    if (map_[iz][iy][ix].getOccupancy() > 0.5f)
    {
      CircularBufferPtr buffer = map_[iz][iy][ix].getPoints();

      begin = buffer->begin();
      end = buffer->end();
      x_off = ((float)ix / resolution_ - (size_ / 2.f));  //+ translation_increment_.x;
      y_off = ((float)iy / resolution_ - (size_ / 2.f));  //+ translation_increment_.y;
      z_off = ((float)iz / resolution_ - (size_ / 2.f));  //+ translation_increment_.z;

      return buffer->size();
    }
    else
      return 0;
  }
  else
    return 0;
}

template <typename PointT>
bool MapLevel<PointT>::getCell(const Eigen::Vector3f& point, GridCellType*& cell_ptr, Eigen::Vector3f& cell_offset,
                               bool include_empty)
{
  Eigen::Vector3f point_with_increment;
  point_with_increment(0) = point(0) + translation_increment_.x;
  point_with_increment(1) = point(1) + translation_increment_.y;
  point_with_increment(2) = point(2) + translation_increment_.z;

  int ix, iy, iz;
  if (calcIndices(point_with_increment, ix, iy, iz))
  {
    if (include_empty || map_[iz][iy][ix].getOccupancy() > 0)
    {
      cell_ptr = &map_[iz][iy][ix];
      cellToMapFrame(cell_offset, ix, iy, iz);
      return true;
    }
  }
  return false;
}

template <typename PointT>
int MapLevel<PointT>::getCellPoints(const pcl::PointXYZ& point, PointCloudPtr cell_points)
{
  int ix, iy, iz;
  if (calcIndices(point, ix, iy, iz))
  {
    if (map_[iz][iy][ix].getOccupancy() > 0.5f)
    {
      CircularBufferPtr buffer = map_[iz][iy][ix].getPoints();

      for (CircularBufferIterator it_cell = buffer->begin(); it_cell != buffer->end(); it_cell++)
      {
        PointT p = (*it_cell);
        cellToMapFrame(*it_cell, p, ix, iy, iz);
        cell_points->push_back(p);
      }
    }
    return cell_points->size();
  }
  else
    return 0;
}

template <typename PointT>
inline int MapLevel<PointT>::getLocalCell(const pcl::PointXYZ& point, CircularBufferIterator& begin,
                                          CircularBufferIterator& end, float& x_off, float& y_off, float& z_off)
{
  int ix, iy, iz;
  if (calcIndices(point, ix, iy, iz))
  {
    if (map_[iz][iy][ix].getOccupancy() > 0.5f)
    {
      CircularBufferPtr buffer = map_[iz][iy][ix].getPoints();

      begin = buffer->begin();
      end = buffer->end();
      x_off = ((float)ix / resolution_ - (size_ / 2.f));  //+ translation_increment_.x;
      y_off = ((float)iy / resolution_ - (size_ / 2.f));  //+ translation_increment_.y;
      z_off = ((float)iz / resolution_ - (size_ / 2.f));  //+ translation_increment_.z;

      return buffer->size();
    }
    else
      return 0;
  }
  else
    return 0;
}

template <typename PointT>
void MapLevel<PointT>::getOccupiedCells(std::vector<pcl::PointXYZ>& cells, bool omit_center )
{
  cells.clear();

  for (size_t iz = 0; iz < map_.size(); iz++)
  {
    for (size_t iy = 0; iy < map_[iz].size(); iy++)
    {
      for (size_t ix = 0; ix < map_[iz][iy].size(); ix++)
      {
        if (map_[iz][iy][ix].getOccupancy() <= OCCUPANCY_UNKNOWN)
          continue;

        if (omit_center && inCenter(ix, iy, iz))
          continue;
	
				Eigen::Vector3f cellOffset;
        cellToMapFrame(cellOffset, ix, iy, iz);
        
        cells.push_back(pcl::PointXYZ(cellOffset(0), cellOffset(1), cellOffset(2))); // TODO: 
      }
    }
  }
}

//TODO: copy pointers..
template <typename PointT>
void MapLevel<PointT>::getOccupiedCellsWithOffset(AlignedCellVector& cells, std::vector<Eigen::Vector3f>& offsets,
                                                  bool omit_center)
{
  cells.clear();
  offsets.clear();

  for (size_t iz = 0; iz < map_.size(); iz++)
  {
    for (size_t iy = 0; iy < map_[iz].size(); iy++)
    {
      for (size_t ix = 0; ix < map_[iz][iy].size(); ix++)
      {
        if (map_[iz][iy][ix].getOccupancy() <= OCCUPANCY_UNKNOWN)
          continue;

        if (omit_center && inCenter(ix, iy, iz))
          continue;

        Eigen::Vector3f cell_offset;
        cellToMapFrame(cell_offset, ix, iy, iz);

        offsets.push_back(cell_offset);
        cells.push_back(map_[iz][iy][ix]);
      }
    }
  }
}

template <typename PointT>
void MapLevel<PointT>::getOccupiedCellsWithOffset(CellPointerVector& cells, std::vector<Eigen::Vector3f>& offsets,
                                                  bool omit_center)
{
  getCellsWithOffset(cells, offsets, omit_center, OCCUPANCY_UNKNOWN);
}

template <typename PointT>
void MapLevel<PointT>::getCellsWithOffset(CellPointerVector& cells, std::vector<Eigen::Vector3f>& offsets,
                                          bool omit_center, float occupancy_threshold)
{
  for (size_t iz = 0; iz < map_.size(); iz++)
  {
    for (size_t iy = 0; iy < map_[iz].size(); iy++)
    {
      for (size_t ix = 0; ix < map_[iz][iy].size(); ix++)
      {
        if (map_[iz][iy][ix].getOccupancy() <= occupancy_threshold)
          continue;

        if (omit_center && inCenter(ix, iy, iz))
          continue;

        Eigen::Vector3f cell_offset;
        cellToMapFrame(cell_offset, ix, iy, iz);

        offsets.push_back(cell_offset);
        cells.push_back(&map_[iz][iy][ix]);
      }
    }
  }
}

template <typename PointT>
void MapLevel<PointT>::getOccupiedCells(AlignedCellVector& cells, bool omit_center)
{
  cells.clear();

  for (size_t iz = 0; iz < map_.size(); iz++)
  {
    for (size_t iy = 0; iy < map_[iz].size(); iy++)
    {
      for (size_t ix = 0; ix < map_[iz][iy].size(); ix++)
      {
        if (map_[iz][iy][ix].getOccupancy() <= 0.f)
          continue;

        if (omit_center && inCenter(ix, iy, iz))
          continue;

        cells.push_back(map_[iz][iy][ix]);
      }
    }
  }
}

template <typename PointT>
void MapLevel<PointT>::getCellPointsDownsampled(PointCloudPtr points, unsigned int points_per_cell)
{
  for (size_t iz = 0; iz < map_.size(); iz++)
  {
    for (size_t iy = 0; iy < map_[iz].size(); iy++)
    {
      for (size_t ix = 0; ix < map_[iz][iy].size(); ix++)
      {
        if (map_[iz][iy][ix].getOccupancy() <= OCCUPANCY_UNKNOWN)
        {
          map_[iz][iy][ix].getPoints()->clear();
          continue;
        }

        CircularBufferPtr cell_points = map_[iz][iy][ix].getPoints();

        unsigned int stride = static_cast<unsigned int>(
            std::min(std::max((int)cell_points->size() / (int)points_per_cell, 1), cell_capacity_));

        unsigned int i = 0;
        CircularBufferIterator it = cell_points->begin();

        while (it != cell_points->end())
        {
          if (i++ == stride)
          {
            PointT p = (*it);
            // transform back to map frame
            cellToMapFrame(*it, p, ix, iy, iz);
            points->push_back(p);
            i = 0;
          }
          it++;
        }
      }
    }
  }
}

template <typename PointT>
bool MapLevel<PointT>::inCenter(int x, int y, int z)
{
  int half_cell_size = map_.size() / 2;
  int quarter_cell_size = map_.size() / 4;

  if ((x >= quarter_cell_size + 1 && x < half_cell_size + quarter_cell_size - 1) &&
      (y >= quarter_cell_size + 1 && y < half_cell_size + quarter_cell_size - 1) &&
      (z >= quarter_cell_size + 1 && z < half_cell_size + quarter_cell_size - 1))
    return true;

  return false;
}

template <typename PointT>
void MapLevel<PointT>::getCellPoints(PointCloudPtr points, bool omit_center)
{
  for (size_t iz = 0; iz < map_.size(); iz++)
  {
    for (size_t iy = 0; iy < map_[iz].size(); iy++)
    {
      for (size_t ix = 0; ix < map_[iz][iy].size(); ix++)
      {
        if (omit_center && inCenter(ix, iy, iz))
        {
          continue;
        }
        if (map_[iz][iy][ix].getOccupancy() <= OCCUPANCY_UNKNOWN)
        {
          map_[iz][iy][ix].getPoints()->clear();
          continue;
        }
        CircularBufferPtr cell_points = map_[iz][iy][ix].getPoints();
        for (CircularBufferIterator it = cell_points->begin(); it != cell_points->end(); it++)
        {
          PointT p = (*it);
          // transform back to map frame
          cellToMapFrame(*it, p, ix, iy, iz);
          points->push_back(p);
        }
      }
    }
  }
}

template <typename PointT>
void MapLevel<PointT>::getCellPointsByScanLabel(PointCloudPtr points, unsigned int scan_id, bool omit_center)
{
  for (size_t iz = 0; iz < map_.size(); iz++)
  {
    for (size_t iy = 0; iy < map_[iz].size(); iy++)
    {
      for (size_t ix = 0; ix < map_[iz][iy].size(); ix++)
      {
        if (omit_center && inCenter(ix, iy, iz))
          continue;

        if (map_[iz][iy][ix].getOccupancy() <= OCCUPANCY_UNKNOWN)
        {
          map_[iz][iy][ix].getPoints()->clear();
          continue;
        }
        CircularBufferPtr cell_points = map_[iz][iy][ix].getPoints();
        for (CircularBufferIterator it = cell_points->begin(); it != cell_points->end(); it++)
        {
          if ((*it).scanNr == scan_id)
          {
            PointT p = (*it);
            // transform back to map frame
            cellToMapFrame(*it, p, ix, iy, iz);
            points->push_back(p);

            points->push_back(p);
          }
        }
      }
    }
  }
}

template <typename PointT>
void MapLevel<PointT>::deleteCellPointsByScanLabel(PointCloudPtr points, unsigned int scan_id, bool omit_center)
{
  for (size_t iz = 0; iz < map_.size(); iz++)
  {
    for (size_t iy = 0; iy < map_[iz].size(); iy++)
    {
      for (size_t ix = 0; ix < map_[iz][iy].size(); ix++)
      {
        if (omit_center && inCenter(ix, iy, iz))
          continue;

        if (map_[iz][iy][ix].getOccupancy() <= OCCUPANCY_UNKNOWN)
        {
          map_[iz][iy][ix].getPoints()->clear();
          continue;
        }
        CircularBufferPtr cell_points = map_[iz][iy][ix].getPoints();
        for (CircularBufferIterator it = cell_points->begin(); it != cell_points->end();)
        {
          if ((*it).scanNr == scan_id)
          {
            PointT p = (*it);
            // transform back to map frame
            cellToMapFrame(*it, p, ix, iy, iz);
            points->push_back(p);

            map_[iz][iy][ix].erasePoint(it);
          }
          else
          {
            it++;
          }
        }
      }
    }
  }
}

template <typename PointT>
void MapLevel<PointT>::getCellPointsByScanLabel(PointCloudPtr points, unsigned int scan_id, unsigned int scan_line_id,
                                                bool omitCenter)
{
  for (size_t iz = 0; iz < map_.size(); iz++)
  {
    for (size_t iy = 0; iy < map_[iz].size(); iy++)
    {
      for (size_t ix = 0; ix < map_[iz][iy].size(); ix++)
      {
        if (omitCenter && inCenter(ix, iy, iz))
          continue;

        if (map_[iz][iy][ix].getOccupancy() <= OCCUPANCY_UNKNOWN)
        {
          map_[iz][iy][ix].getPoints()->clear();
          continue;
        }
        CircularBufferPtr cell_points = map_[iz][iy][ix].getPoints();
        for (CircularBufferIterator it = cell_points->begin(); it != cell_points->end(); it++)
        {
          if ((*it).scanNr == scan_id && (*it).scanlineNr == scan_line_id)
          {
            PointT p = (*it);
            // transform back to map frame
            cellToMapFrame(*it, p, ix, iy, iz);
            points->push_back(p);
          }
        }
      }
    }
  }
}

template <typename PointT>
unsigned int MapLevel<PointT>::getNumCellPoints()
{
	unsigned int num = 0;
  for (size_t iz = 0; iz < map_.size(); iz++)
  {
    for (size_t iy = 0; iy < map_[iz].size(); iy++)
    {
      for (size_t ix = 0; ix < map_[iz][iy].size(); ix++)
      {
        if (map_[iz][iy][ix].getOccupancy() > OCCUPANCY_UNKNOWN)
        {
          num += map_[iz][iy][ix].getPoints()->size();
        }
      }
    }
  }
  return num;
}

template <typename PointT>
bool MapLevel<PointT>::getCell(const Eigen::Vector3f& point, std::vector<GridCellType*>& cell_ptrs,
                               std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& cell_offsets,
                               std::vector<int>& levels, int level, int neighbors)
{
  int max_point = size_ * resolution_;

  Eigen::Vector3f point_with_increment;
  point_with_increment(0) = point(0) + translation_increment_.x;
  point_with_increment(1) = point(1) + translation_increment_.y;
  point_with_increment(2) = point(2) + translation_increment_.z;

  int ix, iy, iz, jx, jy, jz;
  bool retVal = false;

  if (calcIndices(point_with_increment, ix, iy, iz))
  {
    GridCellType* cell_ptr;
    Eigen::Vector3f cell_offset;
    for (int dx = -neighbors; dx <= neighbors; dx++)
    {
      for (int dy = -neighbors; dy <= neighbors; dy++)
      {
        for (int dz = -neighbors; dz <= neighbors; dz++)
        {
          jx = ix + dx;
          jy = iy + dy;
          jz = iz + dz;

          if (jx >= 0 && jy >= 0 && jz >= 0 && jx < max_point && jy < max_point && jz < max_point)
          {
            if (map_[jz][jy][jx].getOccupancy() > OCCUPANCY_UNKNOWN)
            {
              cell_ptr = &map_[jz][jy][jx];
              cellToMapFrame(cell_offset, jx, jy, jz);

              cell_ptrs.push_back(cell_ptr);
              cell_offsets.push_back(cell_offset);
              levels.push_back(level);

              retVal = true;
            }
          }
          else
          {
            // use next coarser resolution at the resolution borders
            cell_ptrs.clear();
            cell_offsets.clear();
            levels.clear();
            return false;
          }
        }
      }
    }
  }

  return retVal;
}

template <typename PointT>
void MapLevel<PointT>::set(const PointT& point_in_map_frame, bool update_occupancy)
{
  int x = 0;
  int y = 0;
  int z = 0;

  PointT point_in_cell_frame = point_in_map_frame;

  if (toCellCoordinate(point_in_map_frame, point_in_cell_frame, x, y, z))
  {
    // reset cell's point list on set to avoid check on decreaseAll()
    if (update_occupancy)
    {
      if (map_[z][y][x].getOccupancy() <= OCCUPANCY_UNKNOWN)
        map_[z][y][x].getPoints()->clear();

      map_[z][y][x].setOccupancy(1.f);
    }
    map_[z][y][x].addPoint(point_in_cell_frame);
  }
}

template <typename PointT>
void MapLevel<PointT>::insertRay(const PointT& p, const Eigen::Matrix4f& sensor_transform)
{
  PointT p_map = p;
  p_map.x += translation_increment_.x;
  p_map.y += translation_increment_.y;
  p_map.z += translation_increment_.z;

  Eigen::Vector3f p_origin(sensor_transform(0, 3), sensor_transform(1, 3), sensor_transform(2, 3));
  p_origin(0) += translation_increment_.x;
  p_origin(1) += translation_increment_.y;
  p_origin(2) += translation_increment_.z;

  if (isnan(p.x) || isnan(p.y) || isnan(p.z))
    return;

  Eigen::Vector3f direction = (p_map.getVector3fMap() - p_origin);

  float direction_length = direction.norm();

  direction /= direction_length;

  int step[3];
  double t_max[3];
  double t_delta[3];

  int current_cell_index[3];

  GridCellType* current_cell_ptr;
  Eigen::Vector3f current_cell_offset;
  Eigen::Vector3f current_cell_point = p_origin;

  // check if current cell is in map range
  int x, y, z;
  if (!calcIndices(current_cell_point, x, y, z))
  {
    return;
  }
  current_cell_index[0] = x;
  current_cell_index[1] = y;
  current_cell_index[2] = z;
  if (!getCell(current_cell_point, current_cell_ptr, current_cell_offset, true))
  {
    return;
  }

  GridCellType* endpoint_cell_ptr;
  Eigen::Vector3f endpoint_cell_offset;
  Eigen::Vector3f endpoint_cell_point = p.getVector3fMap();

  bool endpoint_in_map = false;

  // check if endpoint is in map
  if (getCell(endpoint_cell_point, endpoint_cell_ptr, endpoint_cell_offset, true))
  {
    (*endpoint_cell_ptr).addOccupancy(prob_hit_);
    if ((*endpoint_cell_ptr).getOccupancy() > clamping_thresh_max_)
      (*endpoint_cell_ptr).setOccupancy(clamping_thresh_max_);

    //
    PointT p_local = p;
    p_local.x = fmodf((p_map.x), (1.f / resolution_));
    if (p_local.x < 0.f)
      p_local.x += (1.f / resolution_);
    p_local.y = fmodf((p_map.y), (1.f / resolution_));
    if (p_local.y < 0.f)
      p_local.y += (1.f / resolution_);
    p_local.z = fmodf((p_map.z), (1.f / resolution_));
    if (p_local.z < 0.f)
      p_local.z += (1.f / resolution_);

    (*endpoint_cell_ptr).addPoint(p_local);

    (*endpoint_cell_ptr).debug_state_ = GridCellType::OCCUPIED;

    if (endpoint_cell_ptr == current_cell_ptr)
    {
      return;
    }

    endpoint_in_map = true;
  }

  for (unsigned int i = 0; i < 3; ++i)
  {
    // compute step direction
    if (direction(i) > 0.0)
      step[i] = 1;
    else if (direction(i) < 0.0)
      step[i] = -1;
    else
      step[i] = 0;

    // compute t_max, t_delta
    if (step[i] != 0)
    {
      // corner point of cell (in direction of ray)
      double cell_border = current_cell_offset(i);  // TODO
      cell_border += (float)(step[i] * (1.f / resolution_) * 0.5);

      t_max[i] = (cell_border - p_origin(i)) / direction(i);
      t_delta[i] = (1.f / resolution_) / fabs(direction(i));
    }
    else
    {
      t_max[i] = std::numeric_limits<double>::max();
      t_delta[i] = std::numeric_limits<double>::max();
    }
  }

  // incremental phase 
  bool done = false;
  while (!done)
  {
    unsigned int dim;

    // find minimum tMax:
    if (t_max[0] < t_max[1])
    {
      if (t_max[0] < t_max[2])
        dim = 0;
      else
        dim = 2;
    }
    else
    {
      if (t_max[1] < t_max[2])
        dim = 1;
      else
        dim = 2;
    }

    // advance in direction "dim"
    current_cell_index[dim] += step[dim];

    if (current_cell_index[0] >= 0 && current_cell_index[1] >= 0 && current_cell_index[2] >= 0 &&
        current_cell_index[0] < static_cast<int>(map_.size()) && current_cell_index[1] < static_cast<int>(map_.size()) &&
        current_cell_index[2] < static_cast<int>(map_.size()))
    {
      current_cell_ptr = &map_[current_cell_index[2]][current_cell_index[1]][current_cell_index[0]];
      current_cell_offset(0) = ((double)current_cell_index[0] / resolution_ - (size_ / 2.f));
      current_cell_offset(1) = ((double)current_cell_index[1] / resolution_ - (size_ / 2.f));
      current_cell_offset(2) = ((double)current_cell_index[2] / resolution_ - (size_ / 2.f));

      current_cell_offset(0) -= translation_increment_.x;
      current_cell_offset(1) -= translation_increment_.y;
      current_cell_offset(2) -= translation_increment_.z;

      t_max[dim] += t_delta[dim];

      // reached endpoint?
      if (endpoint_in_map && current_cell_ptr == endpoint_cell_ptr)
      {
        done = true;
        break;
      }
      else
      {
        // reached endpoint map coords?
        // dist_from_origin now contains the length of the ray when traveled until the border of the current cell
        double dist_from_origin = std::min(std::min(t_max[0], t_max[1]), t_max[2]);

        // if this is longer than the expected ray length, we should have already hit the cell containing the end point
        if (dist_from_origin + getCellSize() > direction_length)
        {
          done = true;
          break;
        }

        else
        {  
          if (!(*current_cell_ptr).is_end_point_)
          {
            (*current_cell_ptr).addOccupancy(prob_miss_);
            if ((*current_cell_ptr).getOccupancy() < clamping_thresh_min_)
            {
              (*current_cell_ptr).setOccupancy(clamping_thresh_min_);

              (*current_cell_ptr).getPoints()->clear();
              (*current_cell_ptr).debug_state_ = GridCellType::FREE;
            }
          }
        }
      }
    }

    else
    {
      return;
    }
  } 

  return;
}

template <typename PointT>
void MapLevel<PointT>::setUpdateMask(std::vector<std::vector<std::vector<bool>>>& update_mask)
{
  // check dimensions
  assert(update_mask.size() == (size_ * resolution_));
  for (size_t iz = 0; iz < size_ * resolution_; iz++)
  {
    assert(update_mask[iz].size() == (size_ * resolution_));
    for (size_t iy = 0; iy < size_ * resolution_; iy++)
    {
      assert(update_mask[iz][iy].size() == (size_ * resolution_));
    }
  }

  // copy mask
  for (size_t iz = 0; iz < size_ * resolution_; iz++)
  {
    for (size_t iy = 0; iy < size_ * resolution_; iy++)
    {
      for (size_t ix = 0; ix < size_ * resolution_; ix++)
      {
        update_mask_[iz][iy][ix] = update_mask[iz][iy][ix];
      }
    }
  }
}

template <typename PointT>
void MapLevel<PointT>::setConicalUpdateMask(const Eigen::Vector3f& position, const Eigen::Vector3f& orientation,
                                            float angle)
{
  for (size_t iz = 0; iz < update_mask_.size(); iz++)
  {
    for (size_t iy = 0; iy < update_mask_[iz].size(); iy++)
    {
      for (size_t ix = 0; ix < update_mask_[iz][iy].size(); ix++)
      {
        Eigen::Vector3f cellOffset;
        cellOffset(0) = ((double)ix / resolution_ - (size_ / 2.f));
        cellOffset(1) = ((double)iy / resolution_ - (size_ / 2.f));
        cellOffset(2) = ((double)iz / resolution_ - (size_ / 2.f));

        cellOffset(0) -= translation_increment_.x;
        cellOffset(1) -= translation_increment_.y;
        cellOffset(2) -= translation_increment_.z;

        cellOffset(0) += (1.f / resolution_) / 2;
        cellOffset(1) += (1.f / resolution_) / 2;
        cellOffset(2) += (1.f / resolution_) / 2;

        Eigen::Vector3f vec_pt_cyl;
        vec_pt_cyl(0) = cellOffset(0) - position(0);
        vec_pt_cyl(1) = cellOffset(1) - position(1);
        vec_pt_cyl(2) = cellOffset(2) - position(2);

        // project onto cylinder axis
        Eigen::Vector3f point_on_axis;  // orthogonal vector from cylinder axis to point
        float dot_product =
            orientation(0) * vec_pt_cyl(0) + orientation(1) * vec_pt_cyl(1) + orientation(2) * vec_pt_cyl(2);

        float length = orientation.norm();
        float length_on_axis_norm = dot_product / (length * length);

        dot_product = std::max(dot_product, 0.f);
        dot_product = std::min(dot_product, length * length);
        length_on_axis_norm = std::max(length_on_axis_norm, 0.f);
        length_on_axis_norm = std::min(length_on_axis_norm, 1.f);

        if (dot_product > 0.f && dot_product <= length * length)
        {
          point_on_axis(0) = vec_pt_cyl(0) - (length_on_axis_norm)*orientation(0);
          point_on_axis(1) = vec_pt_cyl(1) - (length_on_axis_norm)*orientation(1);
          point_on_axis(2) = vec_pt_cyl(2) - (length_on_axis_norm)*orientation(2);

          float distance_from_axis = point_on_axis.norm();

          float dist_from_angle = tan(angle / 2.f) * length;

          // in cone
          if (distance_from_axis < dist_from_angle)
          {
            if (dot_product >= (length * length) - pow(((1.f / resolution_) / 2), 2))
              update_mask_[iz][iy][ix] = false;
          }
        }
      }
    }
  }
}

template <typename PointT>
void MapLevel<PointT>::retainPoints(int iz, int iy, int ix)
{
  if (coarser_level_)
  {
    Eigen::Vector3f point_in_world;
    point_in_world(0) = ((float)ix / resolution_ - (size_ / 2.f));
    point_in_world(1) = ((float)iy / resolution_ - (size_ / 2.f));
    point_in_world(2) = ((float)iz / resolution_ - (size_ / 2.f));

    point_in_world(0) -= translation_increment_.x;
    point_in_world(1) -= translation_increment_.y;
    point_in_world(2) -= translation_increment_.z;

    GridCellType* cell_ptr = NULL;
    Eigen::Vector3f cell_offset;

    coarser_level_->getCell(point_in_world, cell_ptr, cell_offset);

    if (cell_ptr != NULL)
    {
      for (CircularBufferIterator it = cell_ptr->getPoints()->begin(); it != cell_ptr->getPoints()->end(); it++)
      {
        PointT p = (*it);
        p.x = cell_offset(0) + (*it).x;
        p.y = cell_offset(1) + (*it).y;
        p.z = cell_offset(2) + (*it).z;

        p.x += translation_increment_.x;
        p.y += translation_increment_.y;
        p.z += translation_increment_.z;

        int x = 0;
        int y = 0;
        int z = 0;
        if (calcIndices(p, x, y, z))
        {
          if (x == ix && y == iy && z == iz)
          {
            p.x -= translation_increment_.x;
            p.y -= translation_increment_.y;
            p.z -= translation_increment_.z;
            set(p);
          }
        }
      }
    }
  }
}

template <typename PointT>
void MapLevel<PointT>::setCoarserLevel(MapLevelPtr coarser_level)
{
  coarser_level_ = coarser_level;
}

template <typename PointT>
void MapLevel<PointT>::setOccupancyParameters(float clamping_thresh_min, float clamping_thresh_max, float prob_hit,
                                              float prob_miss)
{
  clamping_thresh_min_ = clamping_thresh_min;
  clamping_thresh_max_ = clamping_thresh_max;
  prob_hit_ = prob_hit;
  prob_miss_ = prob_miss;
}

template <typename PointT>
void MapLevel<PointT>::setAllEndPointFlags(bool end_point)
{
  for (size_t iz = 0; iz < map_.size(); iz++)
  {
	  for (size_t iy = 0; iy < map_[iz].size(); iy++)
	  {
	    for (size_t ix = 0; ix < map_[iz][iy].size(); ix++)
	      map_[iz][iy][ix].is_end_point_ = end_point;
	  }
  }
}
template <typename PointT>
void MapLevel<PointT>::setEndPointFlag(const PointT& point, const Eigen::Matrix4f& sensor_transform)
{
	if (isnan(point.x) || isnan(point.y) || isnan(point.z))
		return;

	GridCellType* endpoint_cell_ptr;
	Eigen::Vector3f endpoint_cell_offset;
	Eigen::Vector3f endpoint_cell_point = point.getVector3fMap();

	// check if endpoint is in map
	if (getCell(endpoint_cell_point, endpoint_cell_ptr, endpoint_cell_offset, true))
	{
		// TODO: update occupancy etc here?
		(*endpoint_cell_ptr).is_end_point_ = true;
	}
	return;
}


template <typename PointT>
bool MapLevel<PointT>::toCellCoordinate(const PointT& point, PointT& point_in_cell_coords, int& x, int& y, int& z)
{
  PointT point_in_map_coords = point;
  point_in_map_coords.x += translation_increment_.x;
  point_in_map_coords.y += translation_increment_.y;
  point_in_map_coords.z += translation_increment_.z;

	point_in_cell_coords = point;

  x = 0;
  y = 0;
  z = 0;
  if (calcIndices(point_in_map_coords, x, y, z))
  {
    // transform point to grid cell's frame
    point_in_cell_coords.x = fmodf(point_in_map_coords.x, (1.f / resolution_));
    if (point_in_cell_coords.x < 0.f)
      point_in_cell_coords.x += (1.f / resolution_);
    point_in_cell_coords.y = fmodf(point_in_map_coords.y, (1.f / resolution_));
    if (point_in_cell_coords.y < 0.f)
      point_in_cell_coords.y += (1.f / resolution_);
    point_in_cell_coords.z = fmodf(point_in_map_coords.z, (1.f / resolution_));
    if (point_in_cell_coords.z < 0.f)
      point_in_cell_coords.z += (1.f / resolution_);
    return true;
  }
  else
    return false;
}

template <typename PointT>
void MapLevel<PointT>::cellToMapFrame(const PointT& point_in_cell_coords, PointT& point_in_map, int x, int y, int z)
{
  // copy point to make sure custom fields are copied
  point_in_map = point_in_cell_coords;
  // calculate cell's origin and add  to local point
  point_in_map.x = point_in_cell_coords.x + (static_cast<float>(x) / resolution_ - (size_ / 2.f));
  point_in_map.y = point_in_cell_coords.y + (static_cast<float>(y) / resolution_ - (size_ / 2.f));
  point_in_map.z = point_in_cell_coords.z + (static_cast<float>(z) / resolution_ - (size_ / 2.f));
  // add increment
  point_in_map.x -= translation_increment_.x;
  point_in_map.y -= translation_increment_.y;
  point_in_map.z -= translation_increment_.z;
}

template <typename PointT>
void MapLevel<PointT>::cellToMapFrame(Eigen::Vector3f& point_in_map, int x, int y, int z)
{
  // calculate cell's origin and add  to local point
  point_in_map(0) = (static_cast<float>(x) / resolution_ - (size_ / 2.f));
  point_in_map(1) = (static_cast<float>(y) / resolution_ - (size_ / 2.f));
  point_in_map(2) = (static_cast<float>(z) / resolution_ - (size_ / 2.f));
  // add increment
  point_in_map(0) -= translation_increment_.x;
  point_in_map(1) -= translation_increment_.y;
  point_in_map(2) -= translation_increment_.z;
}
}

#endif
