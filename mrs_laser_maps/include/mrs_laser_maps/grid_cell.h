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

//#define DEBUG_CELL_HITS

namespace mrs_laser_maps
{
template <typename PointType = pcl::PointXYZ>
class GridCell
{
public:
  GridCell(int capacity = 100)
    : is_end_point_(false)
    , debug_state_(UNINITIALIZED)
    , points_(new boost::circular_buffer_space_optimized<PointType>(capacity))
    , occupancy_(0.5f)
  {

#ifdef DEBUG_CELL_HITS
    hits_ = 0;
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

  virtual void erasePoint(typename boost::circular_buffer_space_optimized<PointType>::iterator &it)
  {
    it = points_->erase(it);
  }

  virtual const boost::shared_ptr<boost::circular_buffer_space_optimized<PointType>> &getPoints()
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
  boost::shared_ptr<boost::circular_buffer_space_optimized<PointType>> points_;
  float occupancy_;

#ifdef DEBUG_CELL_HITS
  int hits_;
#endif

private:
};
}

#endif
