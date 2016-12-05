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
 *  Author: David Droeschel (droeschel@ais.uni-bonn.de), Jörg Stückler
 */

#include <mrs_laser_mapping/slam_graph.h>

#include <ros/console.h>
#include <ros/assert.h>
#include <pcl/registration/transforms.h>

#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>

#include <g2o/core/optimization_algorithm_levenberg.h>

#define GRADIENT_ITS 100
#define NEWTON_FEAT_ITS 0
#define NEWTON_ITS 5
//#define PRIOR_PROB 0.6
//#define PRIOR_PROB 0.9
#define PRIOR_PROB 0.1

#define LOG_LIKELIHOOD_ADD_THRESHOLD -150000

#define REGISTER_TWICE 0

namespace mrs_laser_mapping
{

SlamGraph::SlamGraph()
{
  ROS_INFO("SLAM::SLAM()");
  srand(time(NULL));

  reference_node_id_ = 0;
  last_transform_.setIdentity();

  // allocating the optimizer
  optimizer_ = new g2o::SparseOptimizer();
  optimizer_->setVerbose(false);
  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* solver = new SlamBlockSolver(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solverLevenberg = new g2o::OptimizationAlgorithmLevenberg(solver);

  optimizer_->setAlgorithm(solverLevenberg);
  
  mrs_laser_maps::RegistrationParameters params;
  params.prior_prob_ = 0.25;
  params.sigma_size_factor_ = 0.45;
  params.soft_assoc_c1_ = 0.9;
  params.soft_assoc_c2_ = 10.0;
  params.associate_once_ = false;
  params.max_iterations_ = 100;
  setRegistrationParameters(params);

  pose_is_close_dist_ = 1.0;
  pose_is_close_angle_ = 0.5;
  pose_is_far_dist_ = 1.7;

  add_nodes_by_distance_ = true;
  add_node_manual_ = false;
  first_frame_ = true;
}

SlamGraph::~SlamGraph()
{
  delete optimizer_;
}


void SlamGraph::setRegistrationParameters(const mrs_laser_maps::RegistrationParameters& params)
{
  surfel_registration_.setRegistrationParameters(params);
}


unsigned int SlamGraph::addKeyFrame(unsigned int kf_prev_id, GraphNodePtr& keyFrame,
                               const Eigen::Matrix4d& transform)
{
  keyFrame->node_id_ = optimizer_->vertices().size();

  // anchor first frame at origin
  if (graph_nodes_.empty())
  {
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId(keyFrame->node_id_);
    v->setEstimate(g2o::SE3Quat());
    v->setFixed(true);
    optimizer_->addVertex(v);
    graph_nodes_.push_back(keyFrame);
    graph_nodes_map_[keyFrame->node_id_] = keyFrame;
  }
  else
  {
    g2o::SE3Quat measurement_mean(Eigen::Quaterniond(transform.block<3, 3>(0, 0)), transform.block<3, 1>(0, 3));

    g2o::VertexSE3* v_prev = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_nodes_[kf_prev_id]->node_id_));

    // create vertex in slam graph for new key frame
    g2o::VertexSE3* v = new g2o::VertexSE3();
    v->setId(keyFrame->node_id_);
    v->setEstimate(v_prev->estimate() * measurement_mean);
    optimizer_->addVertex(v);
    graph_nodes_.push_back(keyFrame);
    graph_nodes_map_[keyFrame->node_id_] = keyFrame;
  }

  return graph_nodes_.size() - 1;
}


bool SlamGraph::addEdge(unsigned int v1_id, unsigned int v2_id)
{
  g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(v1_id));
  g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(v2_id));

  // diff transform from v2 to v1
  Eigen::Matrix4d diffTransform = (v1->estimate().inverse() * v2->estimate()).matrix();

  // add edge to graph
  return addEdge(v1_id, v2_id, diffTransform);
}


bool SlamGraph::addEdge(unsigned int v1_id, unsigned int v2_id, const Eigen::Matrix4d& transform_guess)
{
  Eigen::Matrix4d transform = transform_guess;

  Eigen::Matrix<double, 6, 6> poseCov;

  // TODO: register maps with pose guess from graph
  bool retVal = surfel_registration_.estimateTransformationLevenbergMarquardt(graph_nodes_map_[v1_id]->map_.get(),
                                                             graph_nodes_map_[v2_id]->map_.get(), transform );

  if (!retVal)
    return false;

  retVal = surfel_registration_.estimatePoseCovariance(poseCov, graph_nodes_map_[v1_id]->map_.get(), graph_nodes_map_[v2_id]->map_.get(),
                                      transform);

  if (!retVal)
    return false;

  // add edge to graph
  return addEdge(v1_id, v2_id, transform, poseCov);
}

// returns true, iff node could be added to the cloud

bool SlamGraph::addEdge(unsigned int v1_id, unsigned int v2_id, const Eigen::Matrix4d& transform,
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


bool SlamGraph::poseIsClose(const Eigen::Matrix4d& transform)
{
  // 	double angle = Eigen::AngleAxisd( transform.block< 3, 3 >( 0, 0 ) ).angle();
  double dist = transform.block<3, 1>(0, 3).norm();

  return dist < pose_is_close_dist_;  // fabsf( angle ) < poseIsCloseAngle_ && dist < poseIsCloseDist_;
}


bool SlamGraph::poseIsFar(const Eigen::Matrix4d& transform)
{
  // 	double angle = Eigen::AngleAxisd( transform.block< 3, 3 >( 0, 0 ) ).angle();
  double dist = transform.block<3, 1>(0, 3).norm();

  return dist > pose_is_far_dist_;
}


bool SlamGraph::update(MapPtr local_map, bool localize_only)
{
  pcl::StopWatch stop_watch;
  
  bool add_new_node = false;
  last_update_ == local_map->getLastUpdateTimestamp();

  if (graph_nodes_.empty())
  {
    add_new_node = true;
  }
  else
  {
    Eigen::Matrix4d inc_transform = last_transform_;
    
//     graph_nodes_[reference_node_id_]->map_->evaluateAll();
    
    bool ret_val = false;
    ret_val = surfel_registration_.estimateTransformationLevenbergMarquardt(graph_nodes_[reference_node_id_]->map_.get(), local_map.get(),
                                                          inc_transform);
    ROS_DEBUG_STREAM_NAMED("slam", "registration took: " << stop_watch.getTime() << " with transform " << inc_transform);

    if (ret_val)
    {
      last_transform_ = inc_transform;

      // check for sufficient pose delta to generate a new node
      if (!poseIsClose(last_transform_))
      {
        add_new_node = true;
      }
    }
    else
    {
      ROS_ERROR_STREAM("SLAM: lost track in current frame. reference_node_id_ = " << reference_node_id_);
      // exit( -1 );
      return false;
    }
  }

  if (localize_only)
    ROS_DEBUG_STREAM_NAMED("slam", "map distorted. not adding.");
  
  if ( (add_nodes_by_distance_ || add_node_manual_ || first_frame_) && !localize_only)
  {
    if (add_new_node || add_node_manual_)
    {
      GraphNodePtr node = boost::make_shared<GraphNode>();

      local_map->evaluateAll();
      node->map_ = local_map;
      
      ROS_DEBUG_STREAM_NAMED("slam", "adding new node");

      // TODO: evaluate pose covariance between keyframes..
      Eigen::Matrix<double, 6, 6> pose_cov = Eigen::Matrix<double, 6, 6>::Identity();

      if (!graph_nodes_.empty())
      {
        surfel_registration_.estimatePoseCovariance(pose_cov, graph_nodes_[reference_node_id_]->map_.get(), node->map_.get(),
                                   last_transform_);
      }
      else
        pose_cov.setZero();

      // extend slam graph with vertex for new key frame and with one edge towards the last keyframe..
      unsigned int vertex_id = addKeyFrame(reference_node_id_, node, last_transform_);
      if (optimizer_->vertices().size() > 1)
      {
        if (!addEdge(graph_nodes_[reference_node_id_]->node_id_, graph_nodes_[vertex_id]->node_id_, last_transform_,
                     pose_cov))
        {
          ROS_WARN("WARNING: new key frame not connected to graph!");
          ROS_ASSERT(false);
        }
      }

      ROS_ASSERT(optimizer_->vertices().size() == graph_nodes_.size());

      add_node_manual_ = false;
      first_frame_ = false;
    }

    //  		refine(3,10,100,100);

    // try to match between older key frames (that are close in optimized pose)
    connectClosePoses(false);

    if ( optimizer_->vertices().size() >= 3)
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
  unsigned int old_reference_id = reference_node_id_;
  g2o::VertexSE3* v_ref_old = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_nodes_[old_reference_id]->node_id_));
  Eigen::Matrix4d pose_ref_old = v_ref_old->estimate().matrix();
  Eigen::Matrix4d tracked_pose = pose_ref_old * last_transform_;

  unsigned int bestId = optimizer_->vertices().size() - 1;

  // select closest key frame to current camera pose for further tracking
  // in this way, we do not create unnecessary key frames..
  // 	float bestAngle = std::numeric_limits< float >::max();
  float bestDist = std::numeric_limits<float>::max();
  for (unsigned int kf_id = 0; kf_id < graph_nodes_.size(); kf_id++)
  {
    g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_nodes_[kf_id]->node_id_));

    Eigen::Matrix4d v_pose = v->estimate().matrix();

    Eigen::Matrix4d diffTransform = v_pose.inverse() * tracked_pose;

    double dist = diffTransform.block<3, 1>(0, 3).norm();

    if (/*poseIsClose(diffTransform)*/ dist < 1.9 && dist < bestDist)
    {
      //			bestAngle = angle;
      bestDist = dist;
      bestId = kf_id;
    }
  }

  // try to add new edge between the two reference
  // if not possible, we keep the old reference frame such that a new key frame will added later that connects the two
  // reference frames
  bool switch_reference_id = true;
  g2o::VertexSE3* v_ref = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_nodes_[reference_node_id_]->node_id_));

  if (switch_reference_id)
  {
    reference_node_id_ = bestId;
    ROS_DEBUG_STREAM_NAMED("slam", "switched reference keyframe");
  }

  // set lastTransform_ to pose wrt reference key frame
  v_ref = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_nodes_[reference_node_id_]->node_id_));
  Eigen::Matrix4d pose_ref = v_ref->estimate().matrix();
  last_transform_ = pose_ref.inverse() * tracked_pose;

  ROS_DEBUG_STREAM_NAMED("slam", "update took: " << stop_watch.getTime() << " ms");
  return true;
}


bool SlamGraph::setPose(const Eigen::Matrix4d& poseUpdate)
{
  unsigned int bestId = optimizer_->vertices().size() - 1;

  // select closest key frame to current camera pose for further tracking
  // in this way, we do not create unnecessary key frames..
  // 	float bestAngle = std::numeric_limits< float >::max();
  float bestDist = std::numeric_limits<float>::max();
  for (unsigned int kf_id = 0; kf_id < graph_nodes_.size(); kf_id++)
  {
    g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_nodes_[kf_id]->node_id_));

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
  bool switch_reference_id = true;
  g2o::VertexSE3* v_ref = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_nodes_[reference_node_id_]->node_id_));

  if (switch_reference_id)
  {
    reference_node_id_ = bestId;
    ROS_DEBUG_STREAM_NAMED("slam", "switching reference keyframe id ");
  }

  // set lastTransform_ to pose wrt reference key frame
  v_ref = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(graph_nodes_[reference_node_id_]->node_id_));
  Eigen::Matrix4d pose_ref = v_ref->estimate().matrix();
  last_transform_ = pose_ref.inverse() * poseUpdate;

  return true;
}


void SlamGraph::connectClosePoses(bool random)
{
  // random == true: randomly check only one vertex, the closer, the more probable the check
  if (random)
  {
    const double sigma2_dist = 0.7 * 0.7;
    const double sigma2_angle = 0.3 * 0.3;

    for (unsigned int kf1_id = reference_node_id_; kf1_id <= reference_node_id_; kf1_id++)
    {
      int v1_id = graph_nodes_[kf1_id]->node_id_;
      g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(v1_id));

      std::vector<int> vertices;
      std::vector<double> probs;
      double sum_probs = 0.0;

      for (unsigned int kf2_id = 0; kf2_id < kf1_id; kf2_id++)
      {
        int v2_id = graph_nodes_[kf2_id]->node_id_;
        g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(v2_id));

        // check if edge already exists between the vertices
        bool found_edge = false;
        for (EdgeSet::iterator it = v1->edges().begin(); it != v1->edges().end(); ++it)
        {
          g2o::EdgeSE3* edge = dynamic_cast<g2o::EdgeSE3*>(*it);
          if ((edge->vertices()[0]->id() == v1_id && edge->vertices()[1]->id() == v2_id) ||
              (edge->vertices()[0]->id() == v2_id && edge->vertices()[1]->id() == v1_id))
          {
            found_edge = true;
            break;
          }
        }
        if (found_edge)
          continue;

        // diff transform from v2 to v1
        Eigen::Matrix4d diff_transform = (v1->estimate().inverse() * v2->estimate()).matrix();

        if (poseIsFar(diff_transform))
          continue;

        double angle = Eigen::AngleAxisd(diff_transform.block<3, 3>(0, 0)).angle();
        double dist = diff_transform.block<3, 1>(0, 3).norm();

        // probability of drawing v2 to check for an edge
        double prob_dist = exp(-0.5 * dist * dist / sigma2_dist);
        double prob_angle = exp(-0.5 * angle * angle / sigma2_angle);

        if (prob_dist > 0.1 && prob_angle > 0.1)
        {
          sum_probs += prob_dist * prob_angle;
          probs.push_back(sum_probs);
          vertices.push_back(v2_id);
        }
      }

      if (probs.size() == 0)
        continue;

      // draw random number in [0,sumProbs]
      double check_prob = static_cast<double>(rand()) / static_cast<double>((RAND_MAX + 1.0) * sum_probs);
      for (unsigned int i = 0; i < vertices.size(); i++)
      {
        if (check_prob <= probs[i])
        {
          int v2_id = vertices[i];
          g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(v2_id));
          Eigen::Matrix4d diff_transform = (v1->estimate().inverse() * v2->estimate()).matrix();
          addEdge(v1_id, v2_id, diff_transform);
          break;
        }
      }
    }
  }
  else
  {
    // add all new edges to slam graph
    for (unsigned int kf1_id = 0; kf1_id < graph_nodes_.size(); kf1_id++)
    {
      for (unsigned int kf2_id = 0; kf2_id < kf1_id; kf2_id++)
      {
        int v1_id = graph_nodes_[kf1_id]->node_id_;
        int v2_id = graph_nodes_[kf2_id]->node_id_;
        g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(v1_id));
        g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(v2_id));

        // check if edge already exists between the vertices
        bool found_edge = false;
        for (EdgeSet::iterator it = v1->edges().begin(); it != v1->edges().end(); ++it)
        {
          g2o::EdgeSE3* edge = dynamic_cast<g2o::EdgeSE3*>(*it);
          if ((edge->vertices()[0]->id() == v1_id && edge->vertices()[1]->id() == v2_id) ||
              (edge->vertices()[0]->id() == v2_id && edge->vertices()[1]->id() == v1_id))
          {
            found_edge = true;
            break;
          }
        }
        if (found_edge)
          continue;

        // check if poses close
        // diff transform from v2 to v1
        Eigen::Matrix4d diff_transform = (v1->estimate().inverse() * v2->estimate()).matrix();
        if (poseIsFar(diff_transform))
        {
          // ROS_INFO("pose to far");
          continue;
        }
        ROS_DEBUG_STREAM_NAMED("slam", "adding edge between " << v1_id << " and " << v2_id << " with "
                                                              << diff_transform);
        addEdge(v1_id, v2_id, diff_transform);
      }
    }
  }
}


bool SlamGraph::refineEdge(g2o::EdgeSE3* edge, float register_start_resolution, float register_stop_resolution)
{
  unsigned int v1_id = edge->vertices()[0]->id();
  unsigned int v2_id = edge->vertices()[1]->id();

  g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(v1_id));
  g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(v2_id));

  Eigen::Matrix4d diff_transform = (v1->estimate().inverse() * v2->estimate()).matrix();
  
  // register maps with pose guess from graph
  Eigen::Matrix<double, 6, 6> pose_cov;

  if (graph_nodes_map_.find(v1_id) == graph_nodes_map_.end() || graph_nodes_map_.find(v2_id) == graph_nodes_map_.end())
    return true;  // dont delete this edge!

  
  ROS_DEBUG_STREAM_NAMED("slam", "refineEdge: diff_transform" << diff_transform );
  graph_nodes_map_[v1_id]->map_->evaluateAll();
  graph_nodes_map_[v2_id]->map_->evaluateAll();
  bool ret_val = surfel_registration_.estimateTransformationLevenbergMarquardt(graph_nodes_map_[v1_id]->map_.get(),
                                                             graph_nodes_map_[v2_id]->map_.get(), diff_transform);

  ROS_DEBUG_STREAM_NAMED("slam", " after refine: diff_transform" << diff_transform );

	
  if (!ret_val)
    return false;

  ret_val &= surfel_registration_.estimatePoseCovariance(pose_cov, graph_nodes_map_[v1_id]->map_.get(),
                                       graph_nodes_map_[v2_id]->map_.get(), diff_transform);

  if (ret_val)
  {
    g2o::SE3Quat measurement_mean(Eigen::Quaterniond(diff_transform.block<3, 3>(0, 0)), diff_transform.block<3, 1>(0, 3));
    Eigen::Matrix<double, 6, 6> measurement_information = pose_cov.inverse();

    edge->setMeasurement(measurement_mean);
    edge->setInformation(measurement_information);
    optimizer_->initializeOptimization();
    optimizer_->optimize(10);
    edge->computeError();
    
    
  }

  return ret_val;
}


void SlamGraph::refine(unsigned int refineIterations, unsigned int optimizeIterations, float register_start_resolution,
                  float register_stop_resolution)
{
  if (optimizer_->vertices().size() >= 3)
  {
    for (unsigned int i = 0; i < refineIterations; i++)
    {
      ROS_DEBUG_STREAM_NAMED("slam", "refining " << i << " / " << refineIterations);

      // reestimate all edges in the graph from the current pose estimates in the graph
      std::vector<g2o::EdgeSE3*> remove_edges;
      for (EdgeSet::iterator it = optimizer_->edges().begin(); it != optimizer_->edges().end(); ++it)
      {
        g2o::EdgeSE3* edge = dynamic_cast<g2o::EdgeSE3*>(*it);

        bool retVal = refineEdge(edge, register_start_resolution, register_stop_resolution);

        if (!retVal)
        {
          remove_edges.push_back(edge);
        }
      }

      for (unsigned int j = 0; j < remove_edges.size(); j++)
        optimizer_->removeEdge(remove_edges[j]);

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

void SlamGraph::dumpError()
{
  // dump error of all edges in the slam graph
  std::ofstream outfile("slam_graph_error.dat");

  for (EdgeSet::iterator it = optimizer_->edges().begin(); it != optimizer_->edges().end(); ++it)
  {
    g2o::EdgeSE3* edge = dynamic_cast<g2o::EdgeSE3*>(*it);

    outfile << edge->chi2() << "\n";
  }
}
}