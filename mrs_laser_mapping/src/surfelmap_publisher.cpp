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

#include <mrs_laser_mapping/surfelmap_publisher.h>

#include <tf_conversions/tf_eigen.h>

namespace mrs_laser_mapping
{
SurfelMapPublisher* SurfelMapPublisher::m_instance = 0;

SurfelMapPublisher::SurfelMapPublisher() : m_nodeHandle(), m_lastSurfelMarkerCount(0)
{
  m_markerPublisher = m_nodeHandle.advertise<visualization_msgs::Marker>("/surfel_map/surfels", 100);
  // Jan
  m_pointCloudPublisher = m_nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZ>>("/surfel_map/pointcloud", 10);
  m_pointCloudDownsampledPublisher =
      m_nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZ>>("/surfel_map/pointcloud_downsampled", 10);
  // m_pointCloudPublisher = m_nodeHandle.advertise<sensor_msgs::PointCloud2>("/surfel_map/pointcloud", 10);
  m_pointCloudCuttedPublisher = m_nodeHandle.advertise<sensor_msgs::PointCloud2>("/surfel_map/pointcloud_cut", 10);

  m_scenePointCloudPublisher = m_nodeHandle.advertise<sensor_msgs::PointCloud2>("/surfel_map/scene_pointcloud", 10);
  m_scenePointCloudTransformedPublisher =
      m_nodeHandle.advertise<sensor_msgs::PointCloud2>("/surfel_map/scene_pointcloud_transformed", 10);
  m_correspondencePublisher = m_nodeHandle.advertise<visualization_msgs::Marker>("/surfel_map/correspondences", 10);
}

SurfelMapPublisher::~SurfelMapPublisher()
{
}

SurfelMapPublisher* SurfelMapPublisher::getInstance()
{
  if (m_instance == 0)
    m_instance = new SurfelMapPublisher();
  return m_instance;
}


void SurfelMapPublisher::publishTransformedScenePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  m_scenePointCloudTransformedPublisher.publish(cloud);
}
}
