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

#ifndef _SURFEL_MAP_PUBLISHER_H_
#define _SURFEL_MAP_PUBLISHER_H_

#include <string>
#include <fstream>

#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <mrs_laser_maps/map_multiresolution.h>
#include <mrs_laser_mapping/color_utils.h>

namespace mrs_laser_mapping
{
class SurfelMapPublisher
{
public:
  SurfelMapPublisher();

  ~SurfelMapPublisher();

  static SurfelMapPublisher* getInstance();

  

  template <typename PointType, typename MapType>
  void publishSurfelMarkers(const boost::shared_ptr<MapType>& map);

  template <typename PointType, typename MapType>
  void publishPointCloud(const boost::shared_ptr<MapType>& map);

  template <typename PointType, typename MapType>
  void publishDownsampledPointCloud(const boost::shared_ptr<MapType>& map, int mapSizeDownsampled,
                                    double resolutionDownsampled, int levelsDownsampled, int cellCapacityDownsampled);

  void publishScenePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  void publishTransformedScenePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  template <typename PointType, typename MapType>
  void publishCorrespondences(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source,
                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr target,
                              const boost::shared_ptr<MapType>& map);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

private:
  static SurfelMapPublisher* m_instance;

  ros::NodeHandle m_nodeHandle;

  ros::Publisher m_markerPublisher;

  ros::Publisher m_pointCloudPublisher;
  ros::Publisher m_pointCloudDownsampledPublisher;

  ros::Publisher m_scenePointCloudPublisher;
  ros::Publisher m_scenePointCloudTransformedPublisher;

  ros::Publisher m_correspondencePublisher;

  unsigned int m_lastSurfelMarkerCount;
};
}

#include <mrs_laser_mapping/impl/surfelmap_publisher.hpp>

#endif
