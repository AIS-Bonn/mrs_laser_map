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

#ifndef _GRID_CELL_H_
#define _GRID_CELL_H_

#include <boost/circular_buffer.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

//#include <mrs_laser_maps/cell_buffer.h>

//#define DEBUG_CELL_HITS

namespace mrs_laser_maps
{
  
  
  
template <class PointType = pcl::PointXYZ, class BufferType = boost::circular_buffer_space_optimized<PointType> >
class GridCell 
{
public:
  
  typedef typename boost::shared_ptr<GridCell<PointType, BufferType>> Ptr;
  
  GridCell(int capacity = 100)
    : is_end_point_(false)
    , debug_state_(UNINITIALIZED)
    , points_(boost::make_shared<BufferType>(capacity))
    , occupancy_(0.5f)
  {

#ifdef DEBUG_CELL_HITS
    hits_ = 0;
#endif
  }
  
  GridCell(const GridCell& cell)
    : is_end_point_(cell.is_end_point_)
    , debug_state_(cell.debug_state_)
    , points_(boost::make_shared<BufferType>(*cell.points_.get()))
    , occupancy_(cell.occupancy_)
  {

#ifdef DEBUG_CELL_HITS
    hits_ = cell.hits_;
#endif
  }

  
  virtual ~GridCell(){};

  virtual inline float getOccupancy()
  {
    return occupancy_;
  }

  virtual inline void setOccupancy(const float &occupancy)
  {
    occupancy_ = occupancy;
  }

  virtual inline void substractOccupancy(const float &occupancy)
  {
    occupancy_ -= occupancy;
  }

  virtual inline void addOccupancy(const float &occupancy)
  {
    occupancy_ += occupancy;
  }

  virtual void addPoint(const PointType &p)
  {
    points_->push_front(p);
#ifdef DEBUG_CELL_HITS
    hits_++;
#endif
  }

  virtual inline void addPoint(const boost::shared_ptr<PointType> &p)
  {
  }
  
  virtual void erasePoint(typename BufferType::iterator &it)
  {
    it = points_->erase(it);
  }

  virtual const boost::shared_ptr<BufferType> &getPoints()
  {
    return points_;
  }

  inline int getCellHits()
  {
#ifdef DEBUG_CELL_HITS
    return hits_;
#else
    return 0;
#endif
  }

  bool is_end_point_;

  enum DebugState
  {
    OCCUPIED,
    FREE,
    UNKNOWN,
    UNINITIALIZED
  };

  DebugState debug_state_;

protected:
  boost::shared_ptr<BufferType> points_;
  float occupancy_;

#ifdef DEBUG_CELL_HITS
  int hits_;
#endif

private:
};


//template <>
//inline void GridCell<PointXYZRGBScanLabel, mrs_laser_maps::cell_buffer<PointXYZRGBScanLabel>>::addPoint(const boost::shared_ptr<PointXYZRGBScanLabel>& p)
//  {
//    points_->push_front(p);
//#ifdef DEBUG_CELL_HITS
//    hits_++;
//#endif
//  }
  
// template <>
// void GridCell<PointXYZRGBScanLabel, boost::circular_buffer_space_optimized<boost::shared_ptr<PointXYZRGBScanLabel>>>::addPoint(const PointXYZRGBScanLabel& p)
// {
//     points_->push_front(boost::make_shared<PointXYZRGBScanLabel>(p));
// #ifdef DEBUG_CELL_HITS
//     hits_++;
// #endif
// }
// 
// template <>
// void GridCell<PointXYZRGBScanLabel, boost::circular_buffer_space_optimized<PointXYZRGBScanLabel>>::addPoint(const PointXYZRGBScanLabel& p)
// {
//     points_->push_front(p);
// #ifdef DEBUG_CELL_HITS
//     hits_++;
// #endif
//   }
// 
// template <>
// void GridCell<pcl::PointXYZ, boost::circular_buffer_space_optimized<pcl::PointXYZ>>::addPoint(const pcl::PointXYZ& p)
// {
//   points_->push_front(p);
// #ifdef DEBUG_CELL_HITS
//   hits_++;
// #endif
// }

  
}

#endif
