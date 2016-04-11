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

#include <mrs_laser_mapping/trajectory_publisher.h>

namespace mrs_laser_mapping
{
TrajectoryPublisher* TrajectoryPublisher::m_instance = 0;

TrajectoryPublisher::TrajectoryPublisher() : m_nodeHandle()
{
  m_odometryPublisher = m_nodeHandle.advertise<nav_msgs::Odometry>("laser_odometry", 1);

  m_odometryPublisherMap["/laser_odom"] = m_nodeHandle.advertise<nav_msgs::Odometry>("laser_odometry", 1);
  m_odometryPublisherMap["/vis_odom2"] = m_nodeHandle.advertise<nav_msgs::Odometry>("vis_odometry", 1);
  m_odometryPublisherMap["/mocap"] = m_nodeHandle.advertise<nav_msgs::Odometry>("mocap_odometry", 1);

  m_markerPublisherMap["/laser_odom"] =
      m_nodeHandle.advertise<visualization_msgs::Marker>("trajectory_marker_laser", 10);
  m_markerPublisherMap["/vis_odom2"] = m_nodeHandle.advertise<visualization_msgs::Marker>("trajectory_marker_vis", 10);
  m_markerPublisherMap["/mocap"] = m_nodeHandle.advertise<visualization_msgs::Marker>("trajectory_marker_mocap", 10);

  std::ofstream o;
  //		o.open("/tmp/laser_odometry" );
  //		map<string, ofstream> m_logFiles;
  std::ofstream st;
  // m_fileStreamMap.insert(std::make_pair<std::string, std::ofstream>(string("a"), move(st)));

  ofstream* streamLaser = new ofstream("/tmp/trajectory_laser");
  m_fileStreamMap["/laser_odom"] = streamLaser;

  ofstream* streamVisOdom = new ofstream("/tmp/trajectory_visodom");
  m_fileStreamMap["/vis_odom2"] = streamVisOdom;

  ofstream* streamMocap = new ofstream("/tmp/trajectory_mocap");
  m_fileStreamMap["/mocap"] = streamMocap;

  m_laserDistPlot.open("/tmp/laser_error.txt");
  m_visOdomDistPlot.open("/tmp/vis_odom_error.txt");
}

// Jan : Parameter hinzugefügt für Parallelisierung
TrajectoryPublisher::TrajectoryPublisher(int bagPlayID) : m_nodeHandle()
{
  m_odometryPublisher = m_nodeHandle.advertise<nav_msgs::Odometry>("laser_odometry", 1);

  m_odometryPublisherMap["/laser_odom"] = m_nodeHandle.advertise<nav_msgs::Odometry>("laser_odometry", 1);
  m_odometryPublisherMap["/vis_odom2"] = m_nodeHandle.advertise<nav_msgs::Odometry>("vis_odometry", 1);
  m_odometryPublisherMap["/mocap"] = m_nodeHandle.advertise<nav_msgs::Odometry>("mocap_odometry", 1);

  m_markerPublisherMap["/laser_odom"] =
      m_nodeHandle.advertise<visualization_msgs::Marker>("trajectory_marker_laser", 10);
  m_markerPublisherMap["/vis_odom2"] = m_nodeHandle.advertise<visualization_msgs::Marker>("trajectory_marker_vis", 10);
  m_markerPublisherMap["/mocap"] = m_nodeHandle.advertise<visualization_msgs::Marker>("trajectory_marker_mocap", 10);

  std::ofstream o;
  //		o.open("/tmp/laser_odometry" );
  //		map<string, ofstream> m_logFiles;
  std::ofstream st;
  // m_fileStreamMap.insert(std::make_pair<std::string, std::ofstream>(string("a"), move(st)));

  // Jan
  std::stringstream trajectory_laser_with_ID;
  trajectory_laser_with_ID << "/tmp/trajectory_laser_" << bagPlayID;

  ofstream* streamLaser = new ofstream(trajectory_laser_with_ID.str().c_str());
  m_fileStreamMap["/laser_odom"] = streamLaser;

  std::stringstream trajectory_visodom_with_ID;
  trajectory_visodom_with_ID << "/tmp/trajectory_visodom_" << bagPlayID;

  ofstream* streamVisOdom = new ofstream(trajectory_visodom_with_ID.str().c_str());
  m_fileStreamMap["/vis_odom2"] = streamVisOdom;

  std::stringstream trajectory_mocap_with_ID;
  trajectory_mocap_with_ID << "/tmp/trajectory_mocap_" << bagPlayID;

  ofstream* streamMocap = new ofstream(trajectory_mocap_with_ID.str().c_str());
  m_fileStreamMap["/mocap"] = streamMocap;

  //		ofstream* streamLaser = new ofstream("/tmp/trajectory_laser");
  //		m_fileStreamMap["/laser_odom"] = streamLaser;
  //
  //		ofstream* streamVisOdom= new ofstream("/tmp/trajectory_visodom");
  //		m_fileStreamMap["/vis_odom2"] = streamVisOdom;
  //
  //		ofstream* streamMocap= new ofstream("/tmp/trajectory_mocap");
  //		m_fileStreamMap["/mocap"] = streamMocap;

  m_laserDistPlot.open("/tmp/laser_error.txt");
  m_visOdomDistPlot.open("/tmp/vis_odom_error.txt");
}

TrajectoryPublisher::~TrajectoryPublisher()
{
  m_fileStreamMap["/laser_odom"]->close();
  m_fileStreamMap["/vis_odom2"]->close();
  m_fileStreamMap["/mocap"]->close();
}

void TrajectoryPublisher::publishTrajectoryPoint(float x, float y, float z, float q_x, float q_y, float q_z, float q_w,
                                                 ros::Time stamp, std::string frameId, std::string childFrameId)
{
  tf::Quaternion q(q_x, q_y, q_z, q_w);
  publishTrajectoryPoint(x, y, z, q, stamp, frameId, childFrameId);
}

void TrajectoryPublisher::publishTrajectoryPoint(float x, float y, float z, float roll, float pitch, float yaw,
                                                 ros::Time stamp, std::string frameId, std::string childFrameId)
{
  tf::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  publishTrajectoryPoint(x, y, z, q, stamp, frameId, childFrameId);
}

void TrajectoryPublisher::publishTrajectoryPoint(float x, float y, float z, tf::Quaternion q, ros::Time stamp,
                                                 std::string frameId, std::string childFrameId)
{
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(x, y, z));
  transform.setRotation(q);
  publishTrajectoryPoint(transform, stamp, frameId, childFrameId);
}

void TrajectoryPublisher::publishTrajectoryPoint(Eigen::Matrix4f transform, ros::Time stamp, std::string frameId,
                                                 std::string childFrameId, const Eigen::Matrix<double, 6, 6>& poseCov)
{
  float x, y, z, roll, pitch, yaw;
  Eigen::Affine3f trans(transform);
  pcl::getTranslationAndEulerAngles(trans, x, y, z, roll, pitch, yaw);
  tf::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  tf::Transform tfTransform;
  tfTransform.setOrigin(tf::Vector3(x, y, z));
  tfTransform.setRotation(q);

  publishTrajectoryPoint(tfTransform, stamp, frameId, childFrameId, poseCov);
}

void TrajectoryPublisher::publishTransform(tf::Transform transform, ros::Time stamp, std::string frameId,
                                           std::string childFrameId)
{
  m_transformBroadcaster.sendTransform(tf::StampedTransform(transform, stamp, frameId, childFrameId));
}

void TrajectoryPublisher::publishTrajectoryPoint(tf::Transform transform, ros::Time stamp, std::string frameId,
                                                 std::string childFrameId, const Eigen::Matrix<double, 6, 6>& poseCov)
{
  m_transformBroadcaster.sendTransform(tf::StampedTransform(transform, stamp, frameId, childFrameId));
  publishOdometryMsg(transform, stamp, frameId, childFrameId, poseCov);
}

void TrajectoryPublisher::publishOdometryMsg(tf::Transform transform, ros::Time stamp, std::string frameId,
                                             std::string childFrameId, const Eigen::Matrix<double, 6, 6>& poseCov)
{
  // Jan for darpa
  std::string world_frame = "odom";

  // Jan
  //		std::string world_frame = "world";
  // std::string world_frame = "world_corrected";

  double roll, pitch, yaw;
  Eigen::Affine3f eigenTrans;
  //		for ( unsigned int i = 0; i < 3; i++ )
  //			for ( unsigned int j = 0; j < 3; j++ )
  //				eigenTrans (transform.getBasis())(i);
  //
  //		z

  tf::Vector3 origin = transform.getOrigin();
  tf::Matrix3x3 basis = transform.getBasis();
  basis.getRPY(roll, pitch, yaw);

  tf::Quaternion q;
  q.setRPY(roll, pitch, yaw);

  nav_msgs::Odometry::Ptr odometryMsg(new nav_msgs::Odometry);
  odometryMsg->header.stamp = stamp;
  odometryMsg->header.frame_id = frameId;

  odometryMsg->pose.pose.position.x = origin.getX();
  odometryMsg->pose.pose.position.y = origin.getY();
  odometryMsg->pose.pose.position.z = origin.getZ();

  odometryMsg->pose.pose.orientation.x = q.x();
  odometryMsg->pose.pose.orientation.y = q.y();
  odometryMsg->pose.pose.orientation.z = q.z();
  odometryMsg->pose.pose.orientation.w = q.w();

  //		ROS_INFO_STREAM("publishing childFrameid: " << childFrameId << " framid: " << frameId );
  m_odometryPublisherMap[childFrameId].publish(odometryMsg);

  geometry_msgs::PoseStamped poseChildFrame, poseWorldFrame;
  poseChildFrame.header.stamp = stamp;
  poseChildFrame.header.frame_id = frameId;
  poseChildFrame.pose = odometryMsg->pose.pose;

  if (m_transformListener.waitForTransform(world_frame, frameId, stamp, ros::Duration(0.2f)))
  {
    try
    {
      m_transformListener.transformPose(world_frame, poseChildFrame, poseWorldFrame);
      (*m_fileStreamMap[childFrameId]) << stamp << ", " << poseWorldFrame.pose.position.x << ", "
                                       << poseWorldFrame.pose.position.y << ", " << poseWorldFrame.pose.position.z
                                       << ", " << poseWorldFrame.pose.orientation.x << ", "
                                       << poseWorldFrame.pose.orientation.y << ", " << poseWorldFrame.pose.orientation.z
                                       << ", " << poseWorldFrame.pose.orientation.w << std::endl;

      odometryMsg->pose.pose = poseWorldFrame.pose;

      m_transformsMap[childFrameId].push_back(*odometryMsg);
      m_covariancesMap[childFrameId].push_back(poseCov);

      // offset test HACK
      if (childFrameId == "/laser_odom" || childFrameId == "/vis_odom2")
      {
        //			if ( m_transformsMap["/vis_odom2"].size() > 0 ) {
        //				int n = m_transformsMap["/vis_odom2"].size() -1 ;
        //				geometry_msgs::Point p;
        //				p.x = m_transformsMap["/vis_odom2"].at(n).pose.pose.position.x ;
        //				p.y = m_transformsMap["/vis_odom2"].at(n).pose.pose.position.y ;
        //				p.z = m_transformsMap["/vis_odom2"].at(n).pose.pose.position.z ;
        //
        //
        //				m_offsetHackStream << p.x << ", " << p.y << ", " << p.z << std::endl;
        //			}

        /*
         * find closest mocap pose since we are not sure about timing.
         */

        //					geometry_msgs::PointStamped pLaserLaserFrame, pLaserMocapFrame;
        //					pLaserLaserFrame.header.stamp = stamp;
        //					pLaserLaserFrame.header.frame_id = frameId;
        //					pLaserLaserFrame.point.x = origin.getX();
        //					pLaserLaserFrame.point.y = origin.getY();
        //					pLaserLaserFrame.point.z = origin.getZ();
        //
        //					pLaserMocapFrame.header.stamp = stamp;
        //					pLaserMocapFrame.header.frame_id = "/world";
        //
        //					pcl::PointXYZ pLaserMocap;
        //
        //					if ( m_transformListener.waitForTransform( "/world", frameId, stamp, ros::Duration( 0.2f ) ) ) {
        //
        //						try {
        //							m_transformListener.transformPoint( "/world", pLaserLaserFrame, pLaserMocapFrame );
        //							pcl::PointXYZ pLaser;
        //							pLaser.x = pLaserMocapFrame.point.x ;
        //							pLaser.y = pLaserMocapFrame.point.y ;
        //							pLaser.z = pLaserMocapFrame.point.z ;
        //
        //

        float minDist = FLT_MAX;
        //			for ( unsigned int i = 0; i < m_transformsMap["/mocap"].size(); i++ ) {
        //					if ( m_transformsMap["/mocap"].size() > 0 ) {
        //						int i = m_transformsMap["/mocap"].size() -1 ;
        //
        //						pcl::PointXYZ p;
        //						p.x = m_transformsMap["/mocap"].at(i).pose.pose.position.x ;
        //						p.y = m_transformsMap["/mocap"].at(i).pose.pose.position.y ;
        //						p.z = m_transformsMap["/mocap"].at(i).pose.pose.position.z ;
        //
        //						float dist = euclDist  ( pLaser, p ) ; //sqrt ( ( pLaser.x - p.x ) * ( pLaser.x - p.x ) + (
        //pLaser.y - p.y ) * ( pLaser.y - p.y ) + ( pLaser.z - p.z ) * ( pLaser.z - p.z ) );
        //
        //						if ( dist < minDist)
        //							minDist = dist;
        //					}

        Eigen::Matrix4f eigenTransform = getMatrixForTf(m_transformListener, world_frame, "/mocap", stamp);

        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f trans(eigenTransform);
        pcl::getTranslationAndEulerAngles(trans, x, y, z, roll, pitch, yaw);

        minDist = sqrt((poseWorldFrame.pose.position.x - x) * (poseWorldFrame.pose.position.x - x) +
                       (poseWorldFrame.pose.position.y - y) * (poseWorldFrame.pose.position.y - y) +
                       (poseWorldFrame.pose.position.z - z) * (poseWorldFrame.pose.position.z - z));

        if (childFrameId == "/laser_odom")
        {
          m_laserDistPlot << stamp << ", " << minDist << std::endl;
        }
        if (childFrameId == "/vis_odom2")
        {
          m_visOdomDistPlot << stamp << ", " << minDist << std::endl;
        }
        //						}
        //						catch (tf::TransformException ex)
        //						{
        //							ROS_WARN("TrajectoryPublisher: transofrm error: %s", ex.what());
        //						}
        //					} else {
        //						ROS_WARN("TrajectoryPublisher: could not wait for transform for childframe %s",
        //childFrameId.c_str());
        //
        //					}
        //
      }
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("TrajectoryPublisher: transofrm error: %s", ex.what());
    }
  }
  else
  {
    ROS_WARN("TrajectoryPublisher: could not wait for transform for childframe %s", childFrameId.c_str());
  }

  visualization_msgs::Marker points, line_strip;
  points.header.frame_id = line_strip.header.frame_id = world_frame;
  points.header.stamp = line_strip.header.stamp = stamp;
  points.ns = line_strip.ns = "points_and_lines";
  points.action = line_strip.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

  points.id = 0;
  line_strip.id = 1;

  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.03;
  points.scale.y = 0.03;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.025;

  points.color.r = 0.0f;
  points.color.g = 1.0f;
  points.color.a = 1.0f;

  if (childFrameId == "/laser_odom")
  {
    line_strip.color.r = 1.0f;
    line_strip.color.g = 0.0f;
    line_strip.color.b = 0.0f;
    line_strip.color.a = 1.0f;
  }
  else if (childFrameId == "/mocap")
  {
    line_strip.color.r = 1.0f;
    line_strip.color.g = 0.0f;
    line_strip.color.b = 1.0f;
    line_strip.color.a = 1.0f;
    points.color.r = 1.0f;
    points.color.g = 0.0f;
    points.color.b = 0.5f;
    points.color.a = 1.0f;
  }
  else
  {
    line_strip.color.r = 0.0f;
    line_strip.color.g = 0.0f;
    line_strip.color.b = 1.0f;
    line_strip.color.a = 1.0f;
  }

  //		if (childFrameId == "/laser_odom" ) {
  //			ROS_INFO_STREAM("publishing laser odom: " << m_transformsMap[childFrameId].size() );
  //		}

  // 		int markerId = 0;

  // Create the vertices for the points and lines
  for (uint32_t i = 0; i < m_transformsMap[childFrameId].size(); ++i)
  {
    geometry_msgs::Point p;
    p.x = m_transformsMap[childFrameId].at(i).pose.pose.position.x;
    p.y = m_transformsMap[childFrameId].at(i).pose.pose.position.y;
    p.z = m_transformsMap[childFrameId].at(i).pose.pose.position.z;

    points.points.push_back(p);
    line_strip.points.push_back(p);

    //			Eigen::Matrix<double, 3, 3> cov = m_covariancesMap[childFrameId].at(i).block(0,0,3,3);
    //			if (cov.trace() == 0)
    //				continue;
    //			Eigen::EigenSolver<Eigen::Matrix3d> solver(cov);
    //			Eigen::Matrix<double, 3, 3> eigenvectors = solver.eigenvectors().real();
    //			//				double eigenvalues[3];
    //			Eigen::Matrix<double, 3, 1> eigenvalues = solver.eigenvalues().real();
    //			//				for(int j = 0; j < 3; ++j) {
    //			//					Eigen::Matrix<double, 3, 1> mult = cov * eigenvectors.col(j);
    //			//					eigenvalues[j] = mult(0,0) / eigenvectors.col(j)(0);
    //			//				}
    //			if (eigenvectors.determinant() < 0) {
    //				eigenvectors.col(0)(0) = -eigenvectors.col(0)(0);
    //				eigenvectors.col(0)(1) = -eigenvectors.col(0)(1);
    //				eigenvectors.col(0)(2) = -eigenvectors.col(0)(2);
    //			}
    //			Eigen::Quaternion<double> q = Eigen::Quaternion<double>(eigenvectors);
    //
    //			visualization_msgs::Marker marker;
    //			marker.header.frame_id = "/world";
    //			marker.header.stamp = stamp;
    //			marker.ns = "covariance";
    //			marker.id = markerId++;
    //			marker.type = marker.SPHERE;
    //			marker.action = marker.ADD;
    //			marker.pose.position.x = p.x;
    //			marker.pose.position.y = p.y;
    //			marker.pose.position.z = p.z;
    //			marker.pose.orientation.w = q.w();
    //			marker.pose.orientation.x = q.x();
    //			marker.pose.orientation.y = q.y();
    //			marker.pose.orientation.z = q.z();
    //			marker.scale.x = sqrt(eigenvalues[0]) * 3;
    //			marker.scale.y = sqrt(eigenvalues[1]) * 3;
    //			marker.scale.z = sqrt(eigenvalues[2]) * 3;
    //			marker.color.r = std::min( 1.0, sqrt(eigenvalues[0]) * 3 );
    //			marker.color.g = std::min( 1.0, sqrt(eigenvalues[1]) * 3 );
    //			marker.color.b = std::min( 1.0, sqrt(eigenvalues[2]) * 3 );
    //			marker.color.a = 1.0;
    //
    ////			double dot = surfel.normal_.dot(Eigen::Vector3d(0.,0.,1.));
    ////			if( surfel.normal_.norm() > 1e-10 )
    ////				dot /= surfel.normal_.norm();
    ////			double angle = acos(fabs(dot));
    ////			double angle_normalized = 2. * angle / M_PI;
    ////			marker.color.r = ColorMapJet::red(angle_normalized); //fabs(surfel.normal_(0));
    ////			marker.color.g = ColorMapJet::green(angle_normalized);
    ////			marker.color.b = ColorMapJet::blue(angle_normalized);
    //
    //			m_markerPublisherMap[childFrameId].publish( marker );
  }

  m_markerPublisherMap[childFrameId].publish(points);
  m_markerPublisherMap[childFrameId].publish(line_strip);
}

Eigen::Matrix4f TrajectoryPublisher::getMatrixForTf(const tf::TransformListener& tfListener,
                                                    const std::string& target_frame, const std::string& source_frame,
                                                    const ros::Time& time)
{
  Eigen::Matrix4f eigenTransform;
  tf::StampedTransform tfTransform;

  try
  {
    ros::Time tm;
    std::string err;

    {
      if (tfListener.waitForTransform(target_frame, source_frame, time, ros::Duration(0.2f)))
      {
        tfListener.lookupTransform(target_frame, source_frame, time, tfTransform);
        pcl_ros::transformAsMatrix(tfTransform, eigenTransform);
      }
    }
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("transofrm error: %s", ex.what());
  }
  return eigenTransform;
}

TrajectoryPublisher* TrajectoryPublisher::getInstance()
{
  if (m_instance == 0)
    m_instance = new TrajectoryPublisher();
  return m_instance;
}

// Jan : Parameter hinzugefügt für Parallelisierung
TrajectoryPublisher* TrajectoryPublisher::getInstance(int bagPlayID)
{
  if (m_instance == 0)
    m_instance = new TrajectoryPublisher(bagPlayID);
  return m_instance;
}
}
