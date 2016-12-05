/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu@cs.tum.edu>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: scan_shadows_filter.cpp,v 1.0 2008/12/04 12:00:00 rusu Exp $
 *
 */

/*
  \author Radu Bogdan Rusu <rusu@cs.tum.edu> Tully Foote <tfoote@willowgarage.com>


*/

#ifndef LASER_SCAN_SHADOWS_FILTER_H
#define LASER_SCAN_SHADOWS_FILTER_H

#include <set>
#include <limits>

#include "filters/filter_base.h"
#include <sensor_msgs/LaserScan.h>
#include <angles/angles.h>

//#include <fstream>
//#include <sstream>

namespace mod_laser_filters
{
/** @b ScanShadowsFilter is a simple filter that filters shadow points in a laser scan line
 */

class ScanShadowsFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:
  double laser_max_range_;        // Used in laser scan projection
  double min_angle_, max_angle_;  // Filter angle threshold
  int window_, neighbors_;
  double max_filter_radius_;
  // int seq_;
  // std::ofstream dist_f_;

  ////////////////////////////////////////////////////////////////////////////////
  ScanShadowsFilter() : max_filter_radius_(std::numeric_limits<double>::max())
  //: seq_( 0 )
  //, dist_f_( "/tmp/dist-filter.txt" )
  {
  }

  /**@b Configure the filter from XML */
  bool configure()
  {
    if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam(std::string("min_angle"), min_angle_))
    {
      ROS_ERROR("Error: ShadowsFilter was not given min_angle.\n");
      return false;
    }
    if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam(std::string("max_angle"), max_angle_))
    {
      ROS_ERROR("Error: ShadowsFilter was not given min_angle.\n");
      return false;
    }
    if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam(std::string("window"), window_))
    {
      ROS_ERROR("Error: ShadowsFilter was not given window.\n");
      return false;
    }
    neighbors_ = 0;  // default value
    if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam(std::string("neighbors"), neighbors_))
    {
      ROS_INFO("Error: ShadowsFilter was not given neighbors.\n");
    }

    if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam(std::string("max_filter_radius"), max_filter_radius_))
    {
      ROS_INFO("Error: ShadowsFilter was not max filter radius.\n");
    }

    return true;
  }

  ////////////////////////////////////////////////////////////////////////////////
  virtual ~ScanShadowsFilter()
  {
  }

  /** @brief calculate the perpendicular angle at the end of r1 to get to r2
   * See http://en.wikipedia.org/wiki/Law_of_cosines */
  inline double getAngleWithViewpoint(float r1, float r2, float included_angle)
  {
    return atan2(r2 * sin(included_angle), r1 - r2 * cos(included_angle));
  }

  ////////////////////////////////////////////////////////////////////////////////
  /** \brief Filter shadow points based on 3 global parameters: min_angle, max_angle
   * and window. {min,max}_angle specify the allowed angle interval (in degrees)
   * between the created lines (see getAngleWithViewPoint). Window specifies how many
   * consecutive measurements to take into account for one point.
   * \param scan_in the input LaserScan message
   * \param scan_out the output LaserScan message
   */
  bool update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out)
  {
    // copy across all data first
    scan_out = scan_in;

    std::set<int> indices_to_delete;

    // std::stringstream ss;
    // ss << "/tmp/filter" << seq_;
    // std::ofstream of( ss.str().c_str() );
    // For each point in the current line scan
    for (unsigned int i = 0; i < scan_in.ranges.size(); i++)
    {
      // of << scan_in.ranges[ i ];
      // if(  scan_in.ranges[ i ] > 1.0 && scan_in.ranges[ i ] < 2.0 )
      // ROS_ERROR_STREAM( "R: " << scan_in.ranges[ i ] << " in " << seq_ - 1 );
      for (int y = -window_; y < window_ + 1; y++)
      {
        int j = i + y;
        if (j < 0 || j >= (int)scan_in.ranges.size() || (int)i == j)
        {  // Out of scan bounds or itself
          // of << " 0";
          continue;
        }

        double angle = abs(angles::to_degrees(
            getAngleWithViewpoint(scan_in.ranges[i], scan_in.ranges[j], y * scan_in.angle_increment)));
        // of << " " << angle;
        if (angle < min_angle_ || angle > max_angle_)
        {
          for (int index = std::max<int>(i - neighbors_, 0);
               index <= std::min<int>(i + neighbors_, (int)scan_in.ranges.size() - 1); index++)
          {
            // if (scan_in.ranges[i] < scan_in.ranges[index]) // delete neighbor if they are farther away (note not
            // self)

            if (scan_in.ranges[index] < max_filter_radius_)
              indices_to_delete.insert(index);
          }
        }
      }
      // of << std::endl;
    }
    // of.close();

    ROS_DEBUG("ScanShadowsFilter removing %d Points from scan with min angle: %.2f, max angle: %.2f, neighbors: %d, "
              "and window: %d",
              (int)indices_to_delete.size(), min_angle_, max_angle_, neighbors_, window_);
    // std::stringstream ss2;
    // ss2 << "/tmp/after" << seq_ - 1;
    // std::ofstream of2( ss2.str().c_str() );
    for (std::set<int>::iterator it = indices_to_delete.begin(); it != indices_to_delete.end(); ++it)
    {
      scan_out.ranges[*it] = scan_out.range_max;
      // scan_out.ranges[*it] = std::numeric_limits<float>::quiet_NaN();  // Failed test to set the ranges to invalid
      // value
    }
    // float min_dist = 60.f;
    // for( size_t i = 0; i < scan_out.ranges.size(); i++ ) {
    // of2 << scan_out.ranges[ i ] << "\n";
    // if( scan_out.ranges[ i ] >= 1. )
    // min_dist = std::min( min_dist, scan_out.ranges[ i ] );
    //}

    // dist_f_ << min_dist << std::endl;

    // of2.close();

    // seq_++;

    return true;
  }

  ////////////////////////////////////////////////////////////////////////////////
};
}

#endif  // LASER_SCAN_SHADOWS_FILTER_H
