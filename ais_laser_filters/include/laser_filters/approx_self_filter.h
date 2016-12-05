
#ifndef APPROX_SELF_FILTER_H
#define APPROX_SELF_FILTER_H

#include <set>

#include "filters/filter_base.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "ros/ros.h"
#include "laser_geometry/laser_geometry.h"

namespace mod_laser_filters
{
/** @b ApproxSelfFilter filters the robot/all points inside a cylinder in a laser scan line
 */

class ApproxSelfFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:
  double laser_max_range_;         // Used in laser scan projection
  double radius_, max_z_, min_z_;  // parameters for selffilter

  ////////////////////////////////////////////////////////////////////////////////
  ApproxSelfFilter() : up_and_running_(false)
  {
  }

  /**@b Configure the filter from XML */
  bool configure()
  {
    if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam(std::string("radius"), radius_))
    {
      ROS_ERROR("Error: ApproxSelfFilter was not given radius.\n");
      return false;
    }
    if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam(std::string("max_z"), max_z_))
    {
      ROS_ERROR("Error: ApproxSelfFilter was not given max_z.\n");
      return false;
    }
    if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam(std::string("min_z"), min_z_))
    {
      ROS_ERROR("Error: ApproxSelfFilter was not given min_z.\n");
      return false;
    }

    return true;
  }

  ////////////////////////////////////////////////////////////////////////////////
  virtual ~ApproxSelfFilter()
  {
  }

  ////////////////////////////////////////////////////////////////////////////////
  /** \brief Filter points inside a cylinder around the baselink
   * \param scan_in the input LaserScan message
   * \param scan_out the output LaserScan message
   */
  bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan)
  {
    filtered_scan = input_scan;
    sensor_msgs::PointCloud laser_cloud;

    try
    {
      projector_.transformLaserScanToPointCloud("/base_link", input_scan, laser_cloud, tf_);
    }
    catch (tf::TransformException& ex)
    {
      if (up_and_running_)
      {
        ROS_WARN_THROTTLE(1, "Dropping Scan: Transform unavailable %s", ex.what());
      }
      else
      {
        ROS_INFO_THROTTLE(.3, "Ignoring Scan: Waiting for TF");
      }
      return false;
    }

    int c_idx = indexChannel(laser_cloud);

    if (c_idx == -1 || laser_cloud.channels[c_idx].values.size() == 0)
    {
      ROS_ERROR("We need an index channel to be able to filter out the footprint");
      return false;
    }

    for (unsigned int i = 0; i < laser_cloud.points.size(); i++)
    {
      if (inFootprint(laser_cloud.points[i]))
      {
        int index = laser_cloud.channels[c_idx].values[i];
        filtered_scan.ranges[index] =
            filtered_scan.range_max + 1.0;  // If so, then make it bigger than max range
      }
    }

    up_and_running_ = true;
    return true;
  }

  int indexChannel(const sensor_msgs::PointCloud& scan_cloud)
  {
    int c_idx = -1;
    for (unsigned int d = 0; d < scan_cloud.channels.size(); d++)
    {
      if (scan_cloud.channels[d].name == "index")
      {
        c_idx = d;
        break;
      }
    }
    return c_idx;
  }

  bool inFootprint(const geometry_msgs::Point32& scan_pt)
  {
    if (sqrt(scan_pt.x * scan_pt.x + scan_pt.y * scan_pt.y) > radius_ || scan_pt.z > max_z_ || scan_pt.z < min_z_)
      return false;
    return true;
  }

private:
  tf::TransformListener tf_;
  laser_geometry::LaserProjection projector_;
  double inscribed_radius_;
  bool up_and_running_;
};
}

#endif  // APPROX_SELF_FILTER_H
