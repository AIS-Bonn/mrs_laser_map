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

#ifndef MULTIRESOLUTION_SURFEL_REGISTRATION_H_
#define MULTIRESOLUTION_SURFEL_REGISTRATION_H_

#include <list>

#include <tbb/tbb.h>

#include <pcl/common/time.h>

#include <mrs_laser_maps/map_point_types.h>
#include <mrs_laser_maps/surfel_map_interface.h>

// takes in two map for which it estimates the rigid transformation with a coarse-to-fine strategy.
namespace mrs_laser_maps
{
struct RegistrationParameters
{
  RegistrationParameters()
  : associate_once_(true)
  , prior_prob_(0.25)
  , sigma_size_factor_(0.45)
  , soft_assoc_c1_(0.9)
  , soft_assoc_c2_(10.0)
  , soft_assoc_c3_(1.0)
  , max_iterations_(100)
  {
  }

  bool associate_once_;
    
  double prior_prob_;

  double sigma_size_factor_;

  double soft_assoc_c1_, soft_assoc_c2_, soft_assoc_c3_;
  
  int max_iterations_;
};
  
class MultiResolutionSurfelRegistration
{
public:
  MultiResolutionSurfelRegistration(const RegistrationParameters& params);
  
  MultiResolutionSurfelRegistration();
  
  ~MultiResolutionSurfelRegistration()
  {
  }

  class SingleAssociation
  {
  public:
    SingleAssociation() : cell_scene_(NULL), cell_model_(NULL), match(0)
    {
    }
    SingleAssociation(mrs_laser_maps::SurfelCellInterface* cell_scene, mrs_laser_maps::SurfelCellInterface* cell_model)
      : cell_scene_(cell_scene), cell_model_(cell_model), match(1)
    {
    }
    ~SingleAssociation()
    {
    }

    mrs_laser_maps::SurfelCellInterface* cell_scene_;
    mrs_laser_maps::SurfelCellInterface* cell_model_;

    double error;
    double weight;
    double level;
    double sigma, inv_sigma2;
    int match;

    Eigen::Vector3d model_mean;
    Eigen::Vector3d scene_mean;

    // for Levenberg-Marquardt
    // (z - f)^T W (z - f)
    Eigen::Vector3d z, f;  //, df_qx, df_qy, df_qz;
    Eigen::Matrix<double, 3, 6> df_dx;
    Eigen::Matrix3d W;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  typedef std::vector<SingleAssociation, Eigen::aligned_allocator<SingleAssociation>> SingleAssociationList;

  class SceneSurfelAssociation
  {
  public:
    SceneSurfelAssociation() : cell_scene_(NULL)
    {
    }
    ~SceneSurfelAssociation()
    {
    }

    mrs_laser_maps::SurfelCellInterface* cell_scene_;

    unsigned int model_points_;

    SingleAssociationList associations_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  typedef std::vector<SceneSurfelAssociation, Eigen::aligned_allocator<SceneSurfelAssociation>> AssociationList;

  class CellInfo
  {
  public:
    mrs_laser_maps::SurfelCellInterface* cell_;
    Eigen::Vector3d offset_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  typedef std::vector<CellInfo, Eigen::aligned_allocator<CellInfo>> CellInfoList;

  struct RegistrationFunctionParameters
  {
    mrs_laser_maps::SurfelMapInterface* model;
    mrs_laser_maps::SurfelMapInterface* scene;
    MultiResolutionSurfelRegistration::CellInfoList scene_cells;

    unsigned int model_num_points_, scene_num_points_;

    double prior_prob;
    double sigma_size_factor;

    double soft_assoc_c1, soft_assoc_c2, soft_assoc_c3;

    Eigen::Matrix4d* transform;
    float lastWSign;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr correspondences_source_points_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr correspondences_target_points_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr distance_field_points_;
  };

  void setRegistrationParameters(const RegistrationParameters& params);

  void associateMapsBreadthFirstParallel(AssociationList& surfelAssociations, mrs_laser_maps::SurfelMapInterface* model, mrs_laser_maps::SurfelMapInterface* scene,
                                         MultiResolutionSurfelRegistration::CellInfoList& sceneCells,
                                         Eigen::Matrix4d& transform, double sigma);

  
  bool estimateTransformationLevenbergMarquardt(mrs_laser_maps::SurfelMapInterface* model, mrs_laser_maps::SurfelMapInterface* scene, Eigen::Matrix4d& transform,
                                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr correspondencesSourcePoints,
                                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr correspondencesTargetPoints,
                                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr distanceFieldPoints,
                                                int maxIterations);
  
  bool estimateTransformationLevenbergMarquardt(mrs_laser_maps::SurfelMapInterface* model, mrs_laser_maps::SurfelMapInterface* scene, Eigen::Matrix4d& transform,
                                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr correspondencesSourcePoints,
                                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr correspondencesTargetPoints,
                                                int maxIterations)
  {
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr distanceFieldPoints = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
     return estimateTransformationLevenbergMarquardt(model, scene, transform, correspondencesSourcePoints,
                                                    correspondencesTargetPoints, distanceFieldPoints, maxIterations);
  }

  bool estimateTransformationLevenbergMarquardt(mrs_laser_maps::SurfelMapInterface* model, mrs_laser_maps::SurfelMapInterface* scene, Eigen::Matrix4d& transform,
                                                int maxIterations)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr correspondencesSourcePoints;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr correspondencesTargetPoints;
    return estimateTransformationLevenbergMarquardt(model, scene, transform, correspondencesSourcePoints,
                                                    correspondencesTargetPoints, maxIterations);
  }

  bool estimateTransformationLevenbergMarquardt(mrs_laser_maps::SurfelMapInterface* model, mrs_laser_maps::SurfelMapInterface* scene, Eigen::Matrix4d& transform,
                                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr correspondencesSourcePoints,
                                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr correspondencesTargetPoints)
  {
    return estimateTransformationLevenbergMarquardt(model, scene, transform, correspondencesSourcePoints,
                                                    correspondencesTargetPoints, max_iterations_);
  }
  
  bool estimateTransformationLevenbergMarquardt(mrs_laser_maps::SurfelMapInterface* model, mrs_laser_maps::SurfelMapInterface* scene, Eigen::Matrix4d& transform)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr correspondencesSourcePoints;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr correspondencesTargetPoints;
    return estimateTransformationLevenbergMarquardt(model, scene, transform, correspondencesSourcePoints,
                                                    correspondencesTargetPoints, max_iterations_);
  }
  
  bool estimatePoseCovariance(Eigen::Matrix<double, 6, 6>& poseCov, mrs_laser_maps::SurfelMapInterface* model, mrs_laser_maps::SurfelMapInterface* scene,
                              Eigen::Matrix4d& transform);
  bool estimatePoseCovarianceUnscented(Eigen::Matrix<double, 6, 6>& poseCov, mrs_laser_maps::SurfelMapInterface* model, mrs_laser_maps::SurfelMapInterface* scene,
                                       Eigen::Matrix4d& transform);

  void setPriorPoseEnabled(bool enabled)
  {
    use_prior_pose_ = enabled;
  }
  void setPriorPose(bool enabled, const Eigen::Matrix<double, 6, 1>& prior_pose_mean,
                    const Eigen::Matrix<double, 6, 1>& prior_pose_variances);

  Eigen::Matrix<double, 6, 1> transform2Pose(const Eigen::Matrix4d& transform, double& qw_sign);
  Eigen::Matrix4d pose2Transform(const Eigen::Matrix<double, 6, 1>& pose, double qw_sign);

 

protected:
  bool registrationErrorFunctionLM(const Eigen::Matrix<double, 6, 1>& x,
                                   MultiResolutionSurfelRegistration::RegistrationFunctionParameters& params, double& f,
                                   MultiResolutionSurfelRegistration::AssociationList& associations);
  bool registrationErrorFunctionWithFirstAndSecondDerivativeLM(
      const Eigen::Matrix<double, 6, 1>& x, MultiResolutionSurfelRegistration::RegistrationFunctionParameters& params,
      double& f, Eigen::Matrix<double, 6, 1>& df, Eigen::Matrix<double, 6, 6>& d2f,
      MultiResolutionSurfelRegistration::AssociationList& associations);

  // exposed parameters
  bool use_prior_pose_;
  Eigen::Matrix<double, 6, 1> prior_pose_mean_;
  Eigen::Matrix<double, 6, 6> prior_pose_invcov_;

  Eigen::Matrix<double, 6, 6> last_cov_;

  double prior_prob_;

  double sigma_size_factor_;

  double soft_assoc_c1_, soft_assoc_c2_, soft_assoc_c3_;

  bool associate_once_;
  
  int max_iterations_;
  tbb::task_scheduler_init init_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};
};

#endif /* MULTIRESOLUTION_SURFEL_REGISTRATION_H_ */
