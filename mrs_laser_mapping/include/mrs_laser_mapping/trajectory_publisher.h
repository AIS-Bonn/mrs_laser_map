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

#ifndef _TRAJECTORY_PUBLISHER_H_
#define _TRAJECTORY_PUBLISHER_H_

#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>

#include <visualization_msgs/Marker.h>

#include <string>
#include <fstream>

using namespace std;

namespace mrs_laser_mapping
{
class TrajectoryPublisher
{
public:
  TrajectoryPublisher();
  // Jan : Parameter hinzugefügt für Parallelisierung
  TrajectoryPublisher(int bagPlayID);

  ~TrajectoryPublisher();

  static TrajectoryPublisher* getInstance();

  // Jan : Parameter hinzugefügt für Parallelisierung
  static TrajectoryPublisher* getInstance(int bagPlayID);

  void publishTrajectoryPoint(float x, float y, float z, float q_x, float q_y, float q_z, float q_w, ros::Time stamp,
                              std::string frameId, std::string childFrameId);

  void publishTrajectoryPoint(float x, float y, float z, float roll, float pitch, float yaw, ros::Time stamp,
                              std::string frameId, std::string childFrameId);

  void publishTrajectoryPoint(float x, float y, float z, tf::Quaternion q, ros::Time stamp, std::string frameId,
                              std::string childFrameId);

  void publishTrajectoryPoint(tf::Transform transform, ros::Time stamp, std::string frameId, std::string childFrameId,
                              const Eigen::Matrix<double, 6, 6>& poseCov = Eigen::Matrix<double, 6, 6>::Zero());

  void publishTrajectoryPoint(Eigen::Matrix4f transform, ros::Time stamp, std::string frameId, std::string childFrameId,
                              const Eigen::Matrix<double, 6, 6>& poseCov = Eigen::Matrix<double, 6, 6>::Zero());

  void publishOdometryMsg(tf::Transform transform, ros::Time stamp, std::string frameId, std::string childFrameId,
                          const Eigen::Matrix<double, 6, 6>& poseCov = Eigen::Matrix<double, 6, 6>::Zero());

  Eigen::Matrix4f getMatrixForTf(const tf::TransformListener& tfListener, const std::string& target_frame,
                                 const std::string& source_frame, const ros::Time& time);

  void publishTransform(tf::Transform transform, ros::Time stamp, std::string frameId, std::string childFrameId);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  inline float euclDist(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2)
  {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
  }

private:
  static TrajectoryPublisher* m_instance;

  ros::NodeHandle m_nodeHandle;

  tf::TransformBroadcaster m_transformBroadcaster;

  tf::TransformListener m_transformListener;

  ros::Publisher m_odometryPublisher;

  std::map<std::string, std::vector<nav_msgs::Odometry>> m_transformsMap;

  std::map<std::string, std::vector<Eigen::Matrix<double, 6, 6>>> m_covariancesMap;

  std::map<std::string, ros::Publisher> m_odometryPublisherMap;

  std::map<std::string, ros::Publisher> m_markerPublisherMap;

  std::map<std::string, std::ofstream*> m_fileStreamMap;

  std::ofstream m_visOdomDistPlot;
  std::ofstream m_laserDistPlot;
};
}

#endif
