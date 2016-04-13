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
 *  Author: Jörg Stückler, David Droeschel (droeschel@ais.uni-bonn.de)
 */

#include <mrsmap/slam.h>

#include <pcl/registration/transforms.h>

#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>

#include <g2o/core/optimization_algorithm_levenberg.h>

using namespace mrs_laser_maps;
using namespace mrsmap;

#define GRADIENT_ITS 100
#define NEWTON_FEAT_ITS 0
#define NEWTON_ITS 5
//#define PRIOR_PROB 0.6
//#define PRIOR_PROB 0.9
#define PRIOR_PROB 0.1

#define LOG_LIKELIHOOD_ADD_THRESHOLD -150000

#define REGISTER_TWICE 0

SLAM::SLAM()
{
  ROS_INFO("SLAM::SLAM()");
  srand(time(NULL));

  referenceKeyFrameId_ = 0;
  lastTransform_.setIdentity();
  lastFrameTransform_.setIdentity();

  // allocating the optimizer
  optimizer_ = new g2o::SparseOptimizer();
  optimizer_->setVerbose(true);
  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* solver = new SlamBlockSolver(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solverLevenberg = new g2o::OptimizationAlgorithmLevenberg(solver);

  optimizer_->setAlgorithm(solverLevenberg);

  maxIterations_ = 100;

  priorProb_ = 0.9;
  softAssocC1_ = 1.0;
  softAssocC2_ = 8;
  sigmaSizeFactor_ = 0.25;
  associateOnce_ = false;

  poseIsCloseDist_ = 1.0;
  poseIsCloseAngle_ = 0.5;
  poseIsFarDist_ = 1.7;

  addKeyFrameByDistance_ = true;
  addKeyFrame_ = false;
  firstFrame_ = true;
}

SLAM::~SLAM()
{
  delete optimizer_;
}

unsigned int SLAM::addKeyFrame(unsigned int kf_prev_id, boost::shared_ptr<KeyFrame>& keyFrame,
                               const Eigen::Matrix4d& transform)
{
  keyFrame->nodeId_ = optimizer_->vertices().size();

  // anchor first frame at origin
  if (keyFrames_.empty())
  {
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId(keyFrame->nodeId_);
    v->setEstimate(g2o::SE3Quat());
    v->setFixed(true);
    optimizer_->addVertex(v);
    keyFrames_.push_back(keyFrame);
    keyFrameNodeMap_[keyFrame->nodeId_] = keyFrame;
  }
  else
  {
    g2o::SE3Quat measurement_mean(Eigen::Quaterniond(transform.block<3, 3>(0, 0)), transform.block<3, 1>(0, 3));

    g2o::VertexSE3* v_prev = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(keyFrames_[kf_prev_id]->nodeId_));

    // create vertex in slam graph for new key frame
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId(keyFrame->nodeId_);
    v->setEstimate(v_prev->estimate() * measurement_mean);
    optimizer_->addVertex(v);
    keyFrames_.push_back(keyFrame);
    keyFrameNodeMap_[keyFrame->nodeId_] = keyFrame;
  }

  return keyFrames_.size() - 1;
}

bool SLAM::addEdge(unsigned int v1_id, unsigned int v2_id)
{
  g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(v1_id));
  g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(v2_id));

  // diff transform from v2 to v1
  Eigen::Matrix4d diffTransform = (v1->estimate().inverse() * v2->estimate()).matrix();

  // add edge to graph
  return addEdge(v1_id, v2_id, diffTransform);
}

bool SLAM::addEdge(unsigned int v1_id, unsigned int v2_id, const Eigen::Matrix4d& transformGuess)
{
  Eigen::Matrix4d transform = transformGuess;

  Eigen::Matrix<double, 6, 6> poseCov;

  // TODO: register maps with pose guess from graph
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr corrSrc;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr corrTgt;
  MultiResolutionSurfelRegistration reg;
  reg.prior_prob_ = priorProb_;
  reg.soft_assoc_c1_ = softAssocC1_;
  reg.soft_assoc_c2_ = softAssocC2_;
  reg.sigma_size_factor_ = sigmaSizeFactor_;
  reg.associate_once_ = associateOnce_;

  bool retVal = reg.estimateTransformationLevenbergMarquardt(keyFrameNodeMap_[v1_id]->map_.get(),
                                                             keyFrameNodeMap_[v2_id]->map_.get(), transform, corrSrc,
                                                             corrTgt, maxIterations_);

  if (!retVal)
    return false;

  retVal = reg.estimatePoseCovariance(poseCov, keyFrameNodeMap_[v1_id]->map_.get(), keyFrameNodeMap_[v2_id]->map_.get(),
                                      transform);

  if (!retVal)
    return false;

  // add edge to graph
  return addEdge(v1_id, v2_id, transform, poseCov);
}

// returns true, iff node could be added to the cloud
bool SLAM::addEdge(unsigned int v1_id, unsigned int v2_id, const Eigen::Matrix4d& transform,
                   const Eigen::Matrix<double, 6, 6>& covariance)
{
  g2o::SE3Quat measurement_mean(Eigen::Quaterniond(transform.block<3, 3>(0, 0)), transform.block<3, 1>(0, 3));
  Eigen::Matrix<double, 6, 6> measurement_information = covariance.inverse();

  g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(v1_id));
  g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(v2_id));

  // create edge between new key frame and previous key frame with the estimated transformation
  g2o::EdgeSE3* edge = new g2o::EdgeSE3();
  edge->vertices()[0] = v1;
  edge->vertices()[1] = v2;
  edge->setMeasurement(measurement_mean);
  edge->setInformation(measurement_information);

  ROS_DEBUG_STREAM_NAMED("slam", "adding edge to optimizer between " << v1 << " and " << v2 << " with mean "
                                                                     << measurement_mean << " and information "
                                                                     << measurement_information);
  return optimizer_->addEdge(edge);
}

bool SLAM::poseIsClose(const Eigen::Matrix4d& transform)
{
  // 	double angle = Eigen::AngleAxisd( transform.block< 3, 3 >( 0, 0 ) ).angle();
  double dist = transform.block<3, 1>(0, 3).norm();

  return dist < poseIsCloseDist_;  // fabsf( angle ) < poseIsCloseAngle_ && dist < poseIsCloseDist_;
}

bool SLAM::poseIsFar(const Eigen::Matrix4d& transform)
{
  // 	double angle = Eigen::AngleAxisd( transform.block< 3, 3 >( 0, 0 ) ).angle();
  double dist = transform.block<3, 1>(0, 3).norm();

  return dist > poseIsFarDist_;
}

bool SLAM::update(boost::shared_ptr<mrs_laser_maps::MapType> view, pcl::PointCloud<MapPointType>::Ptr cloud,
                  bool localizeOnly)
{
  //	const int numPoints = pointCloudIn->points.size();
  //
  //	std::vector< int > indices( numPoints );
  //	for( int i = 0; i < numPoints; i++ )
  //		indices[ i ] = i;

  MultiResolutionSurfelRegistration reg;
  reg.prior_prob_ = priorProb_;
  reg.soft_assoc_c1_ = softAssocC1_;
  reg.soft_assoc_c2_ = softAssocC2_;
  reg.sigma_size_factor_ = sigmaSizeFactor_;
  reg.associate_once_ = associateOnce_;

  // 	ROS_INFO_STREAM( "------------ SLAM::update: " << view->getNumCellPoints() );

  bool generateKeyFrame = false;

  if (keyFrames_.empty())
  {
    generateKeyFrame = true;
  }
  else
  {
    Eigen::Matrix4d incTransform = lastTransform_;

    bool retVal = false;

    //		// TODO!! Remove! should already be evaluated
    //		keyFrames_[ referenceKeyFrameId_ ]->map_.get()->evaluateAll();
    //		view.get()->evaluateAll();
    ros::Time timeBeforeRegister = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr corrSrc;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr corrTgt;
    ROS_DEBUG_STREAM_NAMED("slam", "transform before: " << incTransform);
    retVal = reg.estimateTransformationLevenbergMarquardt(keyFrames_[referenceKeyFrameId_]->map_.get(), view.get(),
                                                          incTransform, corrSrc, corrTgt, maxIterations_);
    ROS_DEBUG_STREAM_NAMED("slam", "transform after: " << incTransform);
    ROS_DEBUG_STREAM_NAMED("slam", "registration took: " << ros::Time::now() - timeBeforeRegister);
    //		retVal = true;
    if (retVal)
    {
      lastTransform_ = incTransform;

      // check for sufficient pose delta to generate a new key frame

      if (!poseIsClose(lastTransform_))
      {
        generateKeyFrame = true;
      }
    }
    else
    {
      ROS_ERROR_STREAM("SLAM: lost track in current frame. referenceKeyFrameId_ = " << referenceKeyFrameId_);
      // exit( -1 );
      return false;
    }
  }

  if (addKeyFrameByDistance_ || addKeyFrame_ || firstFrame_)
  {
    if (generateKeyFrame || addKeyFrame_)
    {
      boost::shared_ptr<KeyFrame> keyFrame = boost::shared_ptr<KeyFrame>(new KeyFrame());

      boost::shared_ptr<mrs_laser_maps::MapType> map =
          boost::shared_ptr<mrs_laser_maps::MapType>(new mrs_laser_maps::MapType(*view.get()));
      map->evaluateAll();
      keyFrame->map_ = map;
      // keyFrame->cloud_ = pcl::PointCloud< pcl::PointXYZ>::ConstPtr(new pcl::PointCloud< pcl::PointXYZ>(cloud) );
      keyFrame->cloud_ = cloud;

      ROS_DEBUG_STREAM_NAMED("slam", "------------generate keyframe");

      // TODO: evaluate pose covariance between keyframes..
      Eigen::Matrix<double, 6, 6> poseCov = Eigen::Matrix<double, 6, 6>::Identity();

      if (!keyFrames_.empty())
      {
        reg.estimatePoseCovariance(poseCov, keyFrames_[referenceKeyFrameId_]->map_.get(), keyFrame->map_.get(),
                                   lastTransform_);

        //  				ROS_INFO_STREAM("slam: covariance " << poseCov << " lastTransform_: " << lastTransform_);

        //			poseCov.setIdentity();
      }
      else
        poseCov.setZero();

      // extend slam graph with vertex for new key frame and with one edge towards the last keyframe..
      unsigned int keyFrameId = addKeyFrame(referenceKeyFrameId_, keyFrame, lastTransform_);
      if (optimizer_->vertices().size() > 1)
      {
        if (!addEdge(keyFrames_[referenceKeyFrameId_]->nodeId_, keyFrames_[keyFrameId]->nodeId_, lastTransform_,
                     poseCov))
        {
          ROS_WARN("WARNING: new key frame not connected to graph!");
          ROS_ASSERT(false);
        }
      }

      ROS_ASSERT(optimizer_->vertices().size() == keyFrames_.size());

      addKeyFrame_ = false;
      firstFrame_ = false;
    }

    //  		refine(3,10,100,100);

    // try to match between older key frames (that are close in optimized pose)
    connectClosePoses(false);

    if (optimizer_->vertices().size() >= 3)
    {
      // optimize slam graph
      ROS_DEBUG_STREAM_NAMED("slam", "optimizing...");
      optimizer_->initializeOptimization();
      optimizer_->optimize(10);
      optimizer_->computeActiveErrors();
      ROS_DEBUG_STREAM_NAMED("slam", optimizer_->vertices().size() << " nodes, " << optimizer_->edges().size()
                                                                   << " edges, "
                                                                   << "chi2: " << optimizer_->chi2());
    }
  }

  // get estimated transform in map frame
  unsigned int oldReferenceId_ = referenceKeyFrameId_;
  g2o::VertexSE3* v_ref_old = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(keyFrames_[oldReferenceId_]->nodeId_));
  Eigen::Matrix4d pose_ref_old = v_ref_old->estimate().matrix();
  Eigen::Matrix4d tracked_pose = pose_ref_old * lastTransform_;

  unsigned int bestId = optimizer_->vertices().size() - 1;

  // select closest key frame to current camera pose for further tracking
  // in this way, we do not create unnecessary key frames..
  // 	float bestAngle = std::numeric_limits< float >::max();
  float bestDist = std::numeric_limits<float>::max();
  for (unsigned int kf_id = 0; kf_id < keyFrames_.size(); kf_id++)
  {
    g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(keyFrames_[kf_id]->nodeId_));

    Eigen::Matrix4d v_pose = v->estimate().matrix();

    Eigen::Matrix4d diffTransform = v_pose.inverse() * tracked_pose;

    double dist = diffTransform.block<3, 1>(0, 3).norm();

    if (poseIsClose(diffTransform) && dist < bestDist)
    {
      //			bestAngle = angle;
      bestDist = dist;
      bestId = kf_id;
    }
  }

  // try to add new edge between the two reference
  // if not possible, we keep the old reference frame such that a new key frame will added later that connects the two
  // reference frames
  bool switchReferenceID = true;
  g2o::VertexSE3* v_ref = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(keyFrames_[referenceKeyFrameId_]->nodeId_));

  if (switchReferenceID)
  {
    referenceKeyFrameId_ = bestId;
    ROS_DEBUG_STREAM_NAMED("slam", "switched reference keyframe");
  }

  // set lastTransform_ to pose wrt reference key frame
  v_ref = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(keyFrames_[referenceKeyFrameId_]->nodeId_));
  Eigen::Matrix4d pose_ref = v_ref->estimate().matrix();
  lastTransform_ = pose_ref.inverse() * tracked_pose;

  return true;
}

bool SLAM::setPose(const Eigen::Matrix4d& poseUpdate)
{
  unsigned int bestId = optimizer_->vertices().size() - 1;

  // select closest key frame to current camera pose for further tracking
  // in this way, we do not create unnecessary key frames..
  // 	float bestAngle = std::numeric_limits< float >::max();
  float bestDist = std::numeric_limits<float>::max();
  for (unsigned int kf_id = 0; kf_id < keyFrames_.size(); kf_id++)
  {
    g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(keyFrames_[kf_id]->nodeId_));

    Eigen::Matrix4d v_pose = v->estimate().matrix();

    Eigen::Matrix4d diffTransform = v_pose.inverse() * poseUpdate;

    //		double angle = Eigen::AngleAxisd( diffTransform.block< 3, 3 >( 0, 0 ) ).angle();
    double dist = diffTransform.block<3, 1>(0, 3).norm();

    if (poseIsClose(diffTransform) && dist < bestDist)
    {
      //			bestAngle = angle;
      bestDist = dist;
      bestId = kf_id;
    }
  }

  // try to add new edge between the two reference
  // if not possible, we keep the old reference frame such that a new key frame will added later that connects the two
  // reference frames
  bool switchReferenceID = true;
  g2o::VertexSE3* v_ref = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(keyFrames_[referenceKeyFrameId_]->nodeId_));

  if (switchReferenceID)
  {
    referenceKeyFrameId_ = bestId;
    ROS_DEBUG_STREAM_NAMED("slam", "switching reference keyframe id ");
  }

  // set lastTransform_ to pose wrt reference key frame
  v_ref = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(keyFrames_[referenceKeyFrameId_]->nodeId_));
  Eigen::Matrix4d pose_ref = v_ref->estimate().matrix();
  lastTransform_ = pose_ref.inverse() * poseUpdate;

  return true;
}

void SLAM::connectClosePoses(bool random)
{
  // random == true: randomly check only one vertex, the closer, the more probable the check
  if (random)
  {
    const double sigma2_dist = 0.7 * 0.7;
    const double sigma2_angle = 0.3 * 0.3;

    for (unsigned int kf1_id = referenceKeyFrameId_; kf1_id <= referenceKeyFrameId_; kf1_id++)
    {
      int v1_id = keyFrames_[kf1_id]->nodeId_;
      g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(v1_id));

      std::vector<int> vertices;
      std::vector<double> probs;
      double sumProbs = 0.0;

      for (unsigned int kf2_id = 0; kf2_id < kf1_id; kf2_id++)
      {
        int v2_id = keyFrames_[kf2_id]->nodeId_;
        g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(v2_id));

        // check if edge already exists between the vertices
        bool foundEdge = false;
        for (EdgeSet::iterator it = v1->edges().begin(); it != v1->edges().end(); ++it)
        {
          g2o::EdgeSE3* edge = dynamic_cast<g2o::EdgeSE3*>(*it);
          if ((edge->vertices()[0]->id() == v1_id && edge->vertices()[1]->id() == v2_id) ||
              (edge->vertices()[0]->id() == v2_id && edge->vertices()[1]->id() == v1_id))
          {
            foundEdge = true;
            break;
          }
        }
        if (foundEdge)
          continue;

        // diff transform from v2 to v1
        Eigen::Matrix4d diffTransform = (v1->estimate().inverse() * v2->estimate()).matrix();

        if (poseIsFar(diffTransform))
          continue;

        double angle = Eigen::AngleAxisd(diffTransform.block<3, 3>(0, 0)).angle();
        double dist = diffTransform.block<3, 1>(0, 3).norm();

        // probability of drawing v2 to check for an edge
        double probDist = exp(-0.5 * dist * dist / sigma2_dist);
        double probAngle = exp(-0.5 * angle * angle / sigma2_angle);

        if (probDist > 0.1 && probAngle > 0.1)
        {
          sumProbs += probDist * probAngle;
          probs.push_back(sumProbs);
          vertices.push_back(v2_id);
        }
      }

      if (probs.size() == 0)
        continue;

      // draw random number in [0,sumProbs]
      double checkProb = static_cast<double>(rand()) / static_cast<double>((RAND_MAX + 1.0) * sumProbs);
      for (unsigned int i = 0; i < vertices.size(); i++)
      {
        if (checkProb <= probs[i])
        {
          int v2_id = vertices[i];
          g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(v2_id));
          Eigen::Matrix4d diffTransform = (v1->estimate().inverse() * v2->estimate()).matrix();
          addEdge(v1_id, v2_id, diffTransform);
          break;
        }
      }
    }
  }
  else
  {
    // add all new edges to slam graph
    for (unsigned int kf1_id = 0; kf1_id < keyFrames_.size(); kf1_id++)
    {
      for (unsigned int kf2_id = 0; kf2_id < kf1_id; kf2_id++)
      {
        int v1_id = keyFrames_[kf1_id]->nodeId_;
        int v2_id = keyFrames_[kf2_id]->nodeId_;
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(v1_id));
        g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(v2_id));

        // check if edge already exists between the vertices
        bool foundEdge = false;
        for (EdgeSet::iterator it = v1->edges().begin(); it != v1->edges().end(); ++it)
        {
          g2o::EdgeSE3* edge = dynamic_cast<g2o::EdgeSE3*>(*it);
          if ((edge->vertices()[0]->id() == v1_id && edge->vertices()[1]->id() == v2_id) ||
              (edge->vertices()[0]->id() == v2_id && edge->vertices()[1]->id() == v1_id))
          {
            foundEdge = true;
            break;
          }
        }
        if (foundEdge)
          continue;

        // check if poses close
        // diff transform from v2 to v1
        Eigen::Matrix4d diffTransform = (v1->estimate().inverse() * v2->estimate()).matrix();
        if (poseIsFar(diffTransform))
        {
          // ROS_INFO("pose to far");
          continue;
        }
        ROS_DEBUG_STREAM_NAMED("slam", "adding edge between " << v1_id << " and " << v2_id << " with "
                                                              << diffTransform);
        addEdge(v1_id, v2_id, diffTransform);
      }
    }
  }
}

bool SLAM::refineEdge(g2o::EdgeSE3* edge, float register_start_resolution, float register_stop_resolution)
{
  unsigned int v1_id = edge->vertices()[0]->id();
  unsigned int v2_id = edge->vertices()[1]->id();

  g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(v1_id));
  g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(v2_id));

  Eigen::Matrix4d diffTransform = (v1->estimate().inverse() * v2->estimate()).matrix();

  // register maps with pose guess from graph
  Eigen::Matrix<double, 6, 6> poseCov;

  if (keyFrameNodeMap_.find(v1_id) == keyFrameNodeMap_.end() || keyFrameNodeMap_.find(v2_id) == keyFrameNodeMap_.end())
    return true;  // dont delete this edge!

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr corrSrc;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr corrTgt;
  MultiResolutionSurfelRegistration reg;
  reg.prior_prob_ = priorProb_;
  reg.soft_assoc_c1_ = softAssocC1_;
  reg.soft_assoc_c2_ = softAssocC2_;
  reg.sigma_size_factor_ = sigmaSizeFactor_;
  reg.associate_once_ = associateOnce_;

  bool retVal = reg.estimateTransformationLevenbergMarquardt(keyFrameNodeMap_[v1_id]->map_.get(),
                                                             keyFrameNodeMap_[v2_id]->map_.get(), diffTransform,
                                                             corrSrc, corrTgt, maxIterations_);

  if (!retVal)
    return false;

  retVal &= reg.estimatePoseCovariance(poseCov, keyFrameNodeMap_[v1_id]->map_.get(),
                                       keyFrameNodeMap_[v2_id]->map_.get(), diffTransform);

  if (retVal)
  {
    g2o::SE3Quat measurement_mean(Eigen::Quaterniond(diffTransform.block<3, 3>(0, 0)), diffTransform.block<3, 1>(0, 3));
    Eigen::Matrix<double, 6, 6> measurement_information = poseCov.inverse();

    edge->setMeasurement(measurement_mean);
    edge->setInformation(measurement_information);
  }

  return retVal;
}

void SLAM::refine(unsigned int refineIterations, unsigned int optimizeIterations, float register_start_resolution,
                  float register_stop_resolution)
{
  if (optimizer_->vertices().size() >= 3)
  {
    for (unsigned int i = 0; i < refineIterations; i++)
    {
      ROS_DEBUG_STREAM_NAMED("slam", "refining " << i << " / " << refineIterations);

      // reestimate all edges in the graph from the current pose estimates in the graph
      std::vector<g2o::EdgeSE3*> removeEdges;
      for (EdgeSet::iterator it = optimizer_->edges().begin(); it != optimizer_->edges().end(); ++it)
      {
        g2o::EdgeSE3* edge = dynamic_cast<g2o::EdgeSE3*>(*it);

        bool retVal = refineEdge(edge, register_start_resolution, register_stop_resolution);

        if (!retVal)
        {
          removeEdges.push_back(edge);
        }
      }

      for (unsigned int j = 0; j < removeEdges.size(); j++)
        optimizer_->removeEdge(removeEdges[j]);

      // reoptimize for 10 iterations
      optimizer_->initializeOptimization();
      optimizer_->optimize(10);
      optimizer_->computeActiveErrors();
    }

    // optimize slam graph
    ROS_DEBUG_STREAM_NAMED("slam", "optimizing...");
    optimizer_->initializeOptimization();
    optimizer_->optimize(optimizeIterations);
    optimizer_->computeActiveErrors();
    ROS_DEBUG_STREAM_NAMED("slam", optimizer_->vertices().size() << " nodes, " << optimizer_->edges().size()
                                                                 << " edges, "
                                                                 << "chi2: " << optimizer_->chi2());
  }
}

void SLAM::dumpError()
{
  // dump error of all edges in the slam graph
  std::ofstream outfile("slam_graph_error.dat");

  for (EdgeSet::iterator it = optimizer_->edges().begin(); it != optimizer_->edges().end(); ++it)
  {
    g2o::EdgeSE3* edge = dynamic_cast<g2o::EdgeSE3*>(*it);

    outfile << edge->chi2() << "\n";
  }
}
