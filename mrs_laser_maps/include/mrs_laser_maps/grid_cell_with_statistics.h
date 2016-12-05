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

#ifndef _GRID_CELL_WITH_STATISTICS_H_
#define _GRID_CELL_WITH_STATISTICS_H_

#include <boost/circular_buffer.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>

#include <mrs_laser_maps/grid_cell.h>
#include <mrs_laser_maps/surfel_map_interface.h>
#include <mrs_laser_maps/surfel.h>

namespace mrs_laser_maps
{

template <class PointType = pcl::PointXYZ, class BufferType = boost::circular_buffer_space_optimized<PointType> >
class GridCellWithStatistics : public GridCell<PointType, BufferType>, public SurfelCellInterface
{
public:
  typedef typename boost::shared_ptr<BufferType> CircularBufferPtr;
  typedef typename BufferType::iterator CircularBufferIterator;

  GridCellWithStatistics(int capacity = 100) : GridCell<PointType, BufferType>(capacity)
  , not_added_(0)
  {
  }

  GridCellWithStatistics(const GridCellWithStatistics& cell) 
  : GridCell<PointType, BufferType>(cell)
  , surfel_(cell.surfel_)
  , not_added_(cell.not_added_)
  {
  }
  
  ~GridCellWithStatistics() 
  {
  }
  virtual void clearCell()
  {
    GridCell<PointType,BufferType>::clearCell();
    surfel_.clear();
  }
  
  void evaluate() 
  {
    surfel_.clear();
    
    mrs_laser_maps::Surfel surfel;
    CircularBufferPtr buffer = this->getPoints();

    for (CircularBufferIterator it_cell = buffer->begin(); it_cell != buffer->end();
         it_cell++)
    {
      Eigen::Matrix<double, 3, 1> pos;
      pos(0) = (*it_cell).x;
      pos(1) = (*it_cell).y;
      pos(2) = (*it_cell).z;

      surfel.add(pos);
    }

    surfel_ += surfel;
    surfel_.evaluate();
  }

  template <typename U = PointType, typename std::enable_if<!std::is_same<U, PointXYZRGBScanLabel>::value, int>::type = 0>  
  void evaluate(unsigned int skipScan){ ROS_ERROR("called empty evaluate");};
  
  template <typename U = PointType, typename std::enable_if<std::is_same<U, PointXYZRGBScanLabel>::value, int>::type = 0>
  void evaluate(unsigned int skipScan)
  {
    surfel_.clear();

    mrs_laser_maps::Surfel surfel;
    CircularBufferPtr buffer = this->getPoints();

    for ( auto it_cell = buffer->begin(); it_cell != buffer->end();
         it_cell++)
    {
      if ((*it_cell).scanNr != skipScan)
      {
        Eigen::Matrix<double, 3, 1> pos;
        pos(0) = (*it_cell).x;
        pos(1) = (*it_cell).y;
        pos(2) = (*it_cell).z;
    
        surfel.add(pos);
      }
    }

    surfel_ += surfel;
    surfel_.evaluate();
  }
   
  void evaluateByScanLine(const std::vector<unsigned int>& labels)
  {
    surfel_.clear();

    mrs_laser_maps::Surfel surfel;
    mrs_laser_maps::Surfel scan_line_surfel;
    bool found_scan_line = false;
    
    CircularBufferPtr buffer = this->getPoints();

    for ( auto it_cell = buffer->begin(); it_cell != buffer->end();
         it_cell++)
    {
      if (std::find(labels.begin(), labels.end(), (*it_cell).scanlineNr) != labels.end())
      {
        Eigen::Matrix<double, 3, 1> pos;
        pos(0) = (*it_cell).x;
        pos(1) = (*it_cell).y;
        pos(2) = (*it_cell).z;
    
        scan_line_surfel.add(pos);
	found_scan_line = true;
      }
      else 
      {
        Eigen::Matrix<double, 3, 1> pos;
        pos(0) = (*it_cell).x;
        pos(1) = (*it_cell).y;
        pos(2) = (*it_cell).z;
    
        surfel.add(pos);
      }
    }
    if (found_scan_line)
    {
      surfel_ += scan_line_surfel;
      surfel_.priorize_weight_ = true;
    
      surfel_.degrade_weight_ = false;
      surfel_.weight_scale_ = 1.0;
    }
    else
    {
      surfel_ += surfel;
      surfel_.weight_scale_ = 0.1;
      surfel_.degrade_weight_ = true;
    }
    surfel_.evaluate();
    
  }
   
  template <typename U = PointType, typename std::enable_if<std::is_same<U, PointXYZRGBScanLabel>::value, int>::type = 0>
  void evaluate(const std::vector<unsigned int>& labels)
  {
    surfel_.clear();

    mrs_laser_maps::Surfel surfel;
    CircularBufferPtr buffer = this->getPoints();

    for ( auto it_cell = buffer->begin(); it_cell != buffer->end();
         it_cell++)
    {
      if (std::find(labels.begin(), labels.end(), (*it_cell).scanNr) != labels.end())
      {
	Eigen::Matrix<double, 3, 1> pos;
	pos(0) = (*it_cell).x;
	pos(1) = (*it_cell).y;
	pos(2) = (*it_cell).z;

	surfel.add(pos);
      }
    }

    surfel_ += surfel;
    surfel_.evaluate();
  } 
   
  void addPoint(const PointType &p)
  {
    GridCell<PointType,BufferType>::addPoint(p);
    surfel_.unevaluate();
  }
   
   
  inline void addPoint(const boost::shared_ptr<PointType> &p)
  {
  }
  
/*  void addPoint(const PointType &p)
  {
    if (surfel_.num_points_ > MIN_SURFEL_POINTS)
    {
      Eigen::Matrix<double, 3, 1> point;
      point(0) = p.x;
      point(1) = p.y;
      point(2) = p.z;
      
      double mdist = sqrt((point - surfel_.mean_).transpose() * surfel_.cov_.inverse() * (point - surfel_.mean_));
      if (mdist > 2.0) 
      {
	ROS_INFO_STREAM_THROTTLE(0.25, "mahalanobis dist: " << mdist << " for point " << point << " and mean " << surfel_.mean_ << " with cov " << surfel_.cov_ << " not added " << not_added_);
	GridCell<PointType>::addPoint(p);
      }
      else 
      {
	not_added_++;
      }
    }
    else
    {
       GridCell<PointType>::addPoint(p);
    }
  }
 */  


  inline bool getPointByLabel(unsigned int label, PointType& point )
  {  
    return false;
  }
  
  inline bool updateByLabel(const PointType& point )
  {  
    return false;
  }
  
  void erasePoint(CircularBufferIterator& it)
  {
    it = this->getPoints()->erase(it);
    surfel_.unevaluate();
  }

  inline const mrs_laser_maps::Surfel& getSurfel() const
  {
    return surfel_;
  }
  
  Surfel surfel_;

  int not_added_;
private:
};

template <>
inline void GridCellWithStatistics<PointXYZRGBScanLabel, mrs_laser_maps::cell_buffer<PointXYZRGBScanLabel>>::addPoint(const boost::shared_ptr<PointXYZRGBScanLabel>& p)
{
  GridCell<PointXYZRGBScanLabel,mrs_laser_maps::cell_buffer<PointXYZRGBScanLabel>>::addPoint(p);
  surfel_.unevaluate();
}

template <>
inline bool GridCellWithStatistics<PointXYZRGBScanLabel, mrs_laser_maps::cell_buffer<PointXYZRGBScanLabel>>::getPointByLabel(unsigned int label, PointXYZRGBScanLabel& point )
{
  
  
  return ( this->getPoints()->find(label, point) );
  
}

template <>
inline bool GridCellWithStatistics<PointXYZRGBScanLabel, mrs_laser_maps::cell_buffer<PointXYZRGBScanLabel>>::updateByLabel(const PointXYZRGBScanLabel& point )
{
  return this->getPoints()->update(point);

}

}

#endif
