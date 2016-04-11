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

namespace mrs_laser_maps
{
	const unsigned int MAX_NUM_SURFELS = 6;
	const double MIN_SURFEL_POINTS = 10.0;
	const double MAX_SURFEL_POINTS = 10000.0;

class Surfel
{
public:
  Surfel()
  {
    clear();
  }

  ~Surfel()
  {
  }

  inline void clear()
  {
    num_points_ = 0.0;
    mean_.setZero();
    cov_.setZero();
    sum_.setZero();
    sum_squares_.setZero();

    up_to_date_ = false;
    unevaluated_ = false;
  }

  inline Surfel& operator+=(const Surfel& rhs)
  {
    if (rhs.num_points_ > 0 && num_points_ < MAX_SURFEL_POINTS)
    {
      // numerically stable one-pass update scheme
      if (num_points_ == 0)
      {
        sum_squares_ = rhs.sum_squares_;
        sum_ = rhs.sum_;
        num_points_ = rhs.num_points_;
      }
      else
      {
        const Eigen::Matrix<double, 3, 1> deltaS = rhs.num_points_ * sum_ - num_points_ * rhs.sum_;
        sum_squares_ +=
            rhs.sum_squares_ +
            1.0 / (num_points_ * rhs.num_points_ * (rhs.num_points_ + num_points_)) * deltaS * deltaS.transpose();
        sum_ += rhs.sum_;
        num_points_ += rhs.num_points_;
      }

      first_view_dir_ = rhs.first_view_dir_;
      up_to_date_ = false;
    }

    return *this;
  }

  inline void add(const Eigen::Matrix<double, 3, 1>& point)
  {
    // numerically stable one-pass update scheme
    if (num_points_ < std::numeric_limits<double>::epsilon())
    {
      sum_ += point;
      num_points_ += 1.0;
      up_to_date_ = false;
    }
    else if (num_points_ < MAX_SURFEL_POINTS)
    {
      const Eigen::Matrix<double, 3, 1> deltaS = (sum_ - num_points_ * point);
      sum_squares_ += 1.0 / (num_points_ * (num_points_ + 1.0)) * deltaS * deltaS.transpose();
      sum_ += point;
      num_points_ += 1.0;
      up_to_date_ = false;
    }
  }

  inline void evaluate()
  {
    double det = 0.0;

    curvature_ = 1.0;

    if (num_points_ > 0)
    {
      mean_ = sum_ / num_points_;
    }

    if (num_points_ >= MIN_SURFEL_POINTS)
    {
      cov_ = sum_squares_ / (num_points_ - 1.0);

      // enforce symmetry..
      cov_(1, 0) = cov_(0, 1);
      cov_(2, 0) = cov_(0, 2);
      cov_(2, 1) = cov_(1, 2);
    }
    
    det = cov_.determinant();

    // not enough points or surfel degenerate
    if (num_points_ < MIN_SURFEL_POINTS || det <= std::numeric_limits<double>::epsilon())
    {
    }
    else
    {
      Eigen::Matrix<double, 3, 1> eigen_values_;
      Eigen::Matrix<double, 3, 3> eigen_vectors_;

      // eigen vectors are stored in the columns
      pcl::eigen33(cov_, eigen_vectors_, eigen_values_);

      normal_ = eigen_vectors_.col(0);
      if (normal_.dot(first_view_dir_) > 0.0)
        normal_ *= -1.0;
    }

    up_to_date_ = true;
    unevaluated_ = false;
  }

  inline void unevaluate()
  {
    //			if( num_points_ > 0.0 ) {
    //
    //				mean_ *= num_points_;
    //				cov_ *= (num_points_-1.0);
    //
    //				unevaluated_ = true;
    //
    //			}
  }

  Eigen::Matrix<double, 3, 1> first_view_dir_;

  double num_points_;

  double curvature_;

  Eigen::Matrix<double, 3, 1> mean_, sum_;
  Eigen::Matrix<double, 3, 1> normal_;
  Eigen::Matrix<double, 3, 3> cov_, sum_squares_;
  bool up_to_date_, unevaluated_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:
};

template <typename PointType>
class GridCellWithStatistics : public GridCell<PointType>
{
public:
  typedef boost::circular_buffer<PointType> CircularBuffer;
  typedef boost::shared_ptr<boost::circular_buffer_space_optimized<PointType>> CircularBufferPtr;
  typedef typename CircularBuffer::iterator CircularBufferIterator;

  GridCellWithStatistics(int capacity = 100) : GridCell<PointType>(capacity)
  {
  }

  void evaluate()
  {
    mrs_laser_maps::Surfel surfel;
    boost::shared_ptr<boost::circular_buffer_space_optimized<PointType>> buffer = this->getPoints();

    for (typename boost::circular_buffer<PointType>::iterator it_cell = buffer->begin(); it_cell != buffer->end();
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

  template <typename U = PointType, typename std::enable_if<std::is_same<U, PointXYZRGBScanLabel>::value, int>::type = 0>
  void evaluate(unsigned int skipScan)
  {
    mrs_laser_maps::Surfel surfel;
    boost::shared_ptr<boost::circular_buffer_space_optimized<PointType>> buffer = this->getPoints();

    for (typename boost::circular_buffer<PointType>::iterator it_cell = buffer->begin(); it_cell != buffer->end();
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

  void erasePoint(CircularBufferIterator& it)
  {
    it = this->getPoints()->erase(it);
  }

  Surfel surfel_;

private:
};
}

#endif
