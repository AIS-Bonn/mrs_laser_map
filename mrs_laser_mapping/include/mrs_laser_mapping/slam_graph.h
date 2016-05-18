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

#ifndef SLAM_GRAPH_H_
#define SLAM_GRAPH_H_

#include <mrs_laser_maps/multiresolution_surfel_registration.h>

#include <tr1/unordered_map>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "g2o/core/sparse_optimizer.h"

#include "g2o/types/slam3d/types_slam3d.h"

#include "g2o/core/block_solver.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearCholmodSolver;
typedef std::tr1::unordered_map<int, g2o::HyperGraph::Vertex*> VertexIDMap;
typedef std::set<g2o::HyperGraph::Edge*> EdgeSet;

#include <mrs_laser_maps/surfel_map_interface.h>

namespace mrs_laser_mapping
{

class GraphNode
{
public:

  typedef boost::shared_ptr<mrs_laser_maps::SurfelMapInterface> MapPtr;

  typedef boost::shared_ptr< GraphNode > Ptr;
  typedef boost::shared_ptr< GraphNode const> ConstPtr;
  
  GraphNode()
  {
    sum_log_likelihood_ = 0.0;
    num_edges_ = 0.0;
  }
  ~GraphNode()
  {
  }

  MapPtr map_;
  unsigned int node_id_;

  double sum_log_likelihood_;
  double num_edges_;

private:
};

class SlamGraph
{
public:
  typedef boost::shared_ptr<SlamGraph> Ptr;

  typedef boost::shared_ptr<mrs_laser_maps::SurfelMapInterface> MapPtr;

  typedef GraphNode::Ptr GraphNodePtr;
  typedef GraphNode::ConstPtr GraphNodeConstPtr;
  
  SlamGraph();
  ~SlamGraph();

  void setRegistrationParameters(const mrs_laser_maps::RegistrationParameters& params);
  
  unsigned int addKeyFrame(unsigned int v_prev_id, GraphNodePtr& keyFrame,
                           const Eigen::Matrix4d& transform);

  bool addEdge(unsigned int v1_id, unsigned int v2_id);
  bool addEdge(unsigned int v1_id, unsigned int v2_id, const Eigen::Matrix4d& transform_guess);
  bool addEdge(unsigned int v1_id, unsigned int v2_id, const Eigen::Matrix4d& transform,
               const Eigen::Matrix<double, 6, 6>& covariance);

  bool poseIsClose(const Eigen::Matrix4d& transform);
  bool poseIsFar(const Eigen::Matrix4d& transform);

  bool update(MapPtr view);
  bool setPose(const Eigen::Matrix4d& pose_update);

  void connectClosePoses(bool random = false);

  bool refineEdge(g2o::EdgeSE3* edge, float register_start_resolution, float register_stop_resolution);
  void refine(unsigned int refineIterations, unsigned int optimizeIterations, float register_start_resolution,
              float register_stop_resolution);

  void dumpError();

  g2o::SparseOptimizer* optimizer_;

  std::vector< GraphNodePtr > graph_nodes_;
  std::map< unsigned int, GraphNodePtr > graph_nodes_map_;
  
  unsigned int reference_node_id_;

  double pose_is_close_dist_;
  double pose_is_close_angle_;

  double pose_is_far_dist_;

  bool add_nodes_by_distance_;
  bool add_node_manual_;
  bool first_frame_;

  // wrt reference key frame pose
  Eigen::Matrix4d last_transform_;


  mrs_laser_maps::MultiResolutionSurfelRegistration surfel_registration_;
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:
};
}

#endif
