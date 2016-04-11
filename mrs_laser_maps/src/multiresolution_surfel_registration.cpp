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

#include <mrs_laser_maps/multiresolution_surfel_registration.h>

#include <deque>

#include <fstream>

using namespace mrs_laser_maps;

#define REGISTRATION_MIN_NUM_SURFELS 0

#define PARALLEL 1

#define SOFTASSIGN 1

MultiResolutionSurfelRegistration::MultiResolutionSurfelRegistration()
{
  use_prior_pose_ = false;
  prior_pose_mean_ = Eigen::Matrix<double, 6, 1>::Zero();
  prior_pose_invcov_ = Eigen::Matrix<double, 6, 6>::Identity();
  associate_once_ = true;
  prior_prob_ = 0.9;
  sigma_size_factor_ = 0.25;

  soft_assoc_c1_ = 1.0;
  soft_assoc_c2_ = 8.0;
  soft_assoc_c3_ = 1.0;
}

void MultiResolutionSurfelRegistration::setPriorPose(bool enabled, const Eigen::Matrix<double, 6, 1>& prior_pose_mean,
                                                     const Eigen::Matrix<double, 6, 1>& prior_pose_variances)
{
  use_prior_pose_ = enabled;
  prior_pose_mean_ = prior_pose_mean;
  prior_pose_invcov_ = Eigen::DiagonalMatrix<double, 6>(prior_pose_variances).inverse();
}

class AssociateFunctor
{
public:
  AssociateFunctor(tbb::concurrent_vector<MultiResolutionSurfelRegistration::SceneSurfelAssociation>* associations,
                   MapType* model, MultiResolutionSurfelRegistration::CellInfoList* scene_cells,
                   const Eigen::Matrix4d& transform, const double sigma, double sigma_size_factor)
  {
    associations_ = associations;
    scene_cells_ = scene_cells;
    model_ = model;
    transform_ = transform;
    transformf_ = transform.cast<float>();
    rotation_ = transform.block<3, 3>(0, 0);
    sigma_ = sigma;
    sigma2_ = sigma * sigma;

    sigma_size_factor_ = sigma_size_factor;
  }

  ~AssociateFunctor()
  {
  }

  void operator()(const tbb::blocked_range<size_t>& r) const
  {
    for (size_t i = r.begin(); i != r.end(); ++i)
      (*this)(scene_cells_->at(i));
  }

  void operator()(MultiResolutionSurfelRegistration::CellInfo& sceneCell) const
  {
    MultiResolutionSurfelRegistration::SceneSurfelAssociation assoc;
    assoc.cell_scene_ = sceneCell.cell_;
    assoc.model_points_ = 0;

    Eigen::Vector4d posH;

    posH.block<3, 1>(0, 0) = sceneCell.offset_ + sceneCell.cell_->surfel_.mean_;
    posH(3, 0) = 1.f;

    Eigen::Vector4d scenePointTransformedH = transform_ * posH;
    Eigen::Vector3d scenePointTransformed = scenePointTransformedH.block<3, 1>(0, 0);

    // TODO: volume query in map
    //		std::vector< mrs_laser_maps::GridCellType* > matchedCells;
    //		std::vector< Eigen::Vector3d, Eigen::aligned_allocator< Eigen::Vector3d > > matchedCellsOffsets;

    std::vector<mrs_laser_maps::GridCellType*> cellPtrs;
    std::vector<int> levels;
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> cellOffsets;

    int neighbors;

#if SOFTASSIGN
    neighbors = 1;
#else
    neighbors = 0;
#endif
    int level = 0;
    if (model_->getCell(scenePointTransformed.cast<float>(), cellPtrs, cellOffsets, levels, neighbors))
    {
      level = levels[0];
    }

    double sigma = sigma_size_factor_ * model_->getCellSize(level);
    double sigma2 = sigma * sigma;

    double inv_sigma2 = 1.0 / sigma2;
    double dist2_threshold = std::numeric_limits<double>::max();  // 3.0*9.0 * sigma2;

    // iterate through neighbors of the directly associated node to eventually find a better match
    for (unsigned int i = 0; i < cellPtrs.size(); i++)
    {
      mrs_laser_maps::GridCellType* cell = cellPtrs[i];
      Eigen::Vector3d cellOffsetModel = cellOffsets[i].cast<double>();

      double dist2 = ((cellOffsetModel + cell->surfel_.mean_) - scenePointTransformed).squaredNorm();

      if (dist2 < dist2_threshold)
      {
        MultiResolutionSurfelRegistration::SingleAssociation singleAssoc(sceneCell.cell_, cell);
        singleAssoc.model_mean = (cellOffsetModel + cell->surfel_.mean_);
        singleAssoc.scene_mean = (sceneCell.offset_ + sceneCell.cell_->surfel_.mean_);
        singleAssoc.level = level;
        //				assoc.weight = pow( 2.0, level );
        singleAssoc.sigma = sigma;
        singleAssoc.inv_sigma2 = inv_sigma2;
        assoc.associations_.push_back(singleAssoc);
        assoc.model_points_ += cell->surfel_.num_points_;
      }
    }

    // push associations
    associations_->push_back(assoc);

  }

  tbb::concurrent_vector<MultiResolutionSurfelRegistration::SceneSurfelAssociation>* associations_;

  MapType* model_;

  Eigen::Matrix4d transform_;
  Eigen::Matrix4f transformf_;
  Eigen::Matrix3d rotation_;

  MultiResolutionSurfelRegistration::CellInfoList* scene_cells_;

  double sigma_, sigma2_;

  double sigma_size_factor_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

void MultiResolutionSurfelRegistration::associateMapsBreadthFirstParallel(
    MultiResolutionSurfelRegistration::AssociationList& surfelAssociations, MapType* model, MapType* scene,
    MultiResolutionSurfelRegistration::CellInfoList& sceneCells, Eigen::Matrix4d& transform, double sigma)
{
  tbb::concurrent_vector<MultiResolutionSurfelRegistration::SceneSurfelAssociation> associations;
  associations.reserve(10 * sceneCells.size());

  ROS_DEBUG_STREAM_NAMED("surfel_registration", "associateMapsBreadthFirstParallel size() " << sceneCells.size());

  AssociateFunctor af(&associations, model, &sceneCells, transform, sigma, sigma_size_factor_);

  if (PARALLEL)
    tbb::parallel_for(tbb::blocked_range<size_t>(0, sceneCells.size(), 1000), af);
  else
    std::for_each(sceneCells.begin(), sceneCells.end(), af);

  //	ROS_ERROR_STREAM( "associations.size() " << associations.size() );

  surfelAssociations.insert(surfelAssociations.end(), associations.begin(), associations.end());
}

class GradientFunctorLM
{
public:
  GradientFunctorLM(MultiResolutionSurfelRegistration::AssociationList* assocList,
                    MultiResolutionSurfelRegistration::RegistrationFunctionParameters& params, double tx, double ty,
                    double tz, double qx, double qy, double qz, double qw, bool derivs)
  {
    derivs_ = derivs;
    params_ = &params;

    assocList_ = assocList;

    currentTransform.setIdentity();
    currentTransform.block<3, 3>(0, 0) = Eigen::Matrix3d(Eigen::Quaterniond(qw, qx, qy, qz));
    currentTransform(0, 3) = tx;
    currentTransform(1, 3) = ty;
    currentTransform(2, 3) = tz;

    currentRotation = Eigen::Matrix3d(currentTransform.block<3, 3>(0, 0));
    currentRotationT = currentRotation.transpose();
    currentTranslation = Eigen::Vector3d(currentTransform.block<3, 1>(0, 3));

    if (derivs)
    {
      // build up derivatives of rotation and translation for the transformation variables
      dt_tx(0) = 1.f;
      dt_tx(1) = 0.f;
      dt_tx(2) = 0.f;
      dt_ty(0) = 0.f;
      dt_ty(1) = 1.f;
      dt_ty(2) = 0.f;
      dt_tz(0) = 0.f;
      dt_tz(1) = 0.f;
      dt_tz(2) = 1.f;

      dR_qx.setZero();
      dR_qx(1, 2) = -2;
      dR_qx(2, 1) = 2;

      dR_qy.setZero();
      dR_qy(0, 2) = 2;
      dR_qy(2, 0) = -2;

      dR_qz.setZero();
      dR_qz(0, 1) = -2;
      dR_qz(1, 0) = 2;
    }
  }

  ~GradientFunctorLM()
  {
  }

  void operator()(const tbb::blocked_range<size_t>& r) const
  {
    for (size_t i = r.begin(); i != r.end(); ++i)
      (*this)((*assocList_)[i]);
  }

  void operator()(MultiResolutionSurfelRegistration::SceneSurfelAssociation& assoc) const
  {
    double prior_prob = params_->prior_prob;  // 0.1 - 0.99 ?

    double num_model_points = 0;
    double num_model_surfels = 0;

    double sigma = 1.0;
    for (unsigned int i = 0; i < assoc.associations_.size(); i++)
    {
      MultiResolutionSurfelRegistration::SingleAssociation& singleAssoc = assoc.associations_[i];
      sigma = singleAssoc.sigma;

      if (singleAssoc.match == 0)
      {
        return;
      }
      const Eigen::Vector3d mean_scene = singleAssoc.scene_mean;
      const Eigen::Vector3d mean_model = singleAssoc.model_mean;

      const Eigen::Matrix3d cov_scene =
          0.0001 * mean_scene.norm() * Eigen::Matrix3d::Identity() + singleAssoc.cell_scene_->surfel_.cov_;
      const Eigen::Matrix3d cov_model = singleAssoc.cell_model_->surfel_.cov_;

      Eigen::Vector4d posH;
      posH.block<3, 1>(0, 0) = mean_scene;
      posH(3, 0) = 1.f;

      const Eigen::Vector4d posH_transformed = currentTransform * posH;

      const Eigen::Vector3d mean_scene_transformed = posH_transformed.block<3, 1>(0, 0);
      const Eigen::Vector3d diff = mean_model - mean_scene_transformed;

      const Eigen::Matrix3d cov =
          cov_model + currentRotation * cov_scene * currentRotationT;  // + 0.001*0.001 * Eigen::Matrix3d::Identity();
      const Eigen::Matrix3d invcov = cov.inverse();

      singleAssoc.error = diff.dot(invcov * diff);

      singleAssoc.z = mean_model;
      singleAssoc.f = mean_scene_transformed;

      const Eigen::Matrix3d cov_ps = cov_model + currentRotation * cov_scene * currentRotationT +
                                     singleAssoc.sigma * singleAssoc.sigma * Eigen::Matrix3d::Identity();
      const Eigen::Matrix3d invcov_ps = cov_ps.inverse();

#if (SOFTASSIGN)

      singleAssoc.weight = singleAssoc.cell_model_->surfel_.num_points_ * params_->soft_assoc_c1 /
                           sqrt(params_->soft_assoc_c2 * M_PI * M_PI * M_PI * cov_ps.determinant()) *
                           exp(-0.5 * diff.dot(invcov_ps * diff));

#else
      singleAssoc.weight = 1.f;
#endif


      num_model_points += singleAssoc.cell_model_->surfel_.num_points_;
      num_model_surfels += 1.0;

      if (derivs_)
      {
        singleAssoc.df_dx.block<3, 1>(0, 0) = dt_tx;
        singleAssoc.df_dx.block<3, 1>(0, 1) = dt_ty;
        singleAssoc.df_dx.block<3, 1>(0, 2) = dt_tz;
        singleAssoc.df_dx.block<3, 1>(0, 3) = dR_qx * mean_scene_transformed;
        singleAssoc.df_dx.block<3, 1>(0, 4) = dR_qy * mean_scene_transformed;
        singleAssoc.df_dx.block<3, 1>(0, 5) = dR_qz * mean_scene_transformed;

        singleAssoc.W = invcov;
      }
    }
    const Eigen::Matrix3d cov_scene = currentRotation * assoc.cell_scene_->surfel_.cov_ * currentRotationT +
                                      sigma * sigma * Eigen::Matrix3d::Identity();

  
    // normalize weights
    double sumWeight =
        prior_prob / (1.0 - prior_prob) * ((double)params_->model_num_points_ / (double)params_->scene_num_points_) *
        params_->soft_assoc_c1 / sqrt(params_->soft_assoc_c2 * M_PI * M_PI * M_PI * cov_scene.determinant());

    for (unsigned int i = 0; i < assoc.associations_.size(); i++)
    {
      if (assoc.associations_[i].match == 0)
      {
        continue;
      }
      sumWeight += assoc.associations_[i].weight;
    }

    if (sumWeight > 0.0)
    {
      double invSumWeight = 1.0 / sumWeight;
      for (unsigned int i = 0; i < assoc.associations_.size(); i++)
      {
        //				MultiResolutionSurfelRegistration::SingleAssociation& singleAssoc = assoc.associations_[i];
        if (assoc.associations_[i].match == 0)
        {
          continue;
        }
        //				assoc.associations_[i].weight *= invSumWeight;
        assoc.associations_[i].weight *= assoc.cell_scene_->surfel_.num_points_ * invSumWeight;
        //				assoc.associations_[i].weight *= assoc.cell_scene_->surfel_.num_points_ *
        //singleAssoc.cell_model_->surfel_.num_points_ * invSumWeight;
        //				std::cout << invSumWeight << std::endl;
      }
    }
  }

  Eigen::Matrix4d currentTransform;

  Eigen::Vector3d currentTranslation;
  Eigen::Vector3d dt_tx, dt_ty, dt_tz;

  Eigen::Matrix3d currentRotation, currentRotationT;
  Eigen::Matrix3d dR_qx, dR_qy, dR_qz;

  MultiResolutionSurfelRegistration::AssociationList* assocList_;

  MultiResolutionSurfelRegistration::RegistrationFunctionParameters* params_;

  bool derivs_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

bool MultiResolutionSurfelRegistration::registrationErrorFunctionLM(
    const Eigen::Matrix<double, 6, 1>& x, MultiResolutionSurfelRegistration::RegistrationFunctionParameters& params,
    double& f, MultiResolutionSurfelRegistration::AssociationList& associations)
{
  double sumError = 0.0;
  double sumWeight = 0.0;

  const double tx = x(0);
  const double ty = x(1);
  const double tz = x(2);
  const double qx = x(3);
  const double qy = x(4);
  const double qz = x(5);
  if (qx * qx + qy * qy + qz * qz > 1.0)
    std::cout << "quaternion not stable!!\n";
  const double qw = params.lastWSign * sqrtf(1.0 - qx * qx - qy * qy - qz * qz);  // retrieve sign from last qw
  GradientFunctorLM gf(&associations, params, tx, ty, tz, qx, qy, qz, qw, false);

  pcl::StopWatch timer;
  timer.reset();
  if (PARALLEL)
    tbb::parallel_for(tbb::blocked_range<size_t>(0, associations.size(), 1000), gf);
  else
    std::for_each(associations.begin(), associations.end(), gf);

  ROS_DEBUG_STREAM_NAMED("surfel_registration", "GradientFunctorLM in registrationErrorFunctionLM took: "
                                                    << timer.getTime() << " for " << associations.size()
                                                    << " associations");
  int cidx = 0;
  if (params.correspondences_source_points_)
  {
    params.correspondences_source_points_->points.reserve(10 * associations.size());
    params.correspondences_target_points_->points.reserve(10 * associations.size());
  }

  timer.reset();

  double numMatches = 0;
  for (MultiResolutionSurfelRegistration::AssociationList::iterator it2 = associations.begin();
       it2 != associations.end(); ++it2)
  {
    //		for( MultiResolutionSurfelRegistration::SingleAssociationList::iterator it = it2->associations_.begin(); it !=
    //it2->associations_.end(); ++it ) {
    for (unsigned int i = 0; i < it2->associations_.size(); i++)
    {
      MultiResolutionSurfelRegistration::SingleAssociation* it = &it2->associations_[i];

      if (!it->match)
        continue;

      float weight = it->weight;
      //		float weight = 1.0 / sqrt(it->error);
      //		float weight = it->weight * exp( -0.5 * (it->z - it->f).squaredNorm() * it->inv_sigma2 );

      sumError += weight * it->error;
      sumWeight += weight;
      numMatches += 1.0;  // nweight;

      if (params.correspondences_source_points_)
      {
        pcl::PointXYZRGB p1;
        pcl::PointXYZRGB p2;

        Eigen::Vector4f pos1;
        pos1.block<3, 1>(0, 0) = it->model_mean.cast<float>();
        pos1(3) = 1.f;
        Eigen::Vector4f pos2;
        pos2.block<3, 1>(0, 0) = it->scene_mean.cast<float>();
        pos2(3) = 1.f;

        p1.x = pos1(0);
        p1.y = pos1(1);
        p1.z = pos1(2);

        p1.r = weight * 255.f;
        p1.g = 0;
        p1.b = (1.f - weight) * 255.f;

        Eigen::Vector4d pos;
        pos.block<3, 1>(0, 0) = pos2.block<3, 1>(0, 0).cast<double>();
        pos(3, 0) = 1.f;

        const Eigen::Vector4f pos_src = pos2;  // gf.currentTransform * pos;

        p2.x = pos_src[0];
        p2.y = pos_src[1];
        p2.z = pos_src[2];

        p2.r = weight * 255.f;
        p2.g = 0;
        p2.b = (1.f - weight) * 255.f;

        params.correspondences_source_points_->points.push_back(p1);
        params.correspondences_target_points_->points.push_back(p2);
      }
      cidx++;
    }
  }

  ROS_DEBUG_STREAM_NAMED("surfel_registration", "took: " << timer.getTime() << " for " << numMatches << " matches from "
                                                         << cidx << " single assocs");

  //	ROS_ERROR_STREAM( "num matches " << numMatches );

  if (params.correspondences_source_points_)
  {
    params.correspondences_source_points_->points.resize(cidx);
    params.correspondences_target_points_->points.resize(cidx);
  }

  if (sumWeight <= 1e-10)
  {
    sumError = std::numeric_limits<double>::max();
    return false;
  }
  else if (numMatches < REGISTRATION_MIN_NUM_SURFELS)
  {
    sumError = std::numeric_limits<double>::max();
    ROS_ERROR_STREAM("not enough surfels for robust matching " << numMatches);
    return false;
  }

  f = sumError / sumWeight;

  if (use_prior_pose_)
  {
    f += (prior_pose_mean_ - x).transpose() * prior_pose_invcov_ * (prior_pose_mean_ - x);
  }

  return true;
}

bool MultiResolutionSurfelRegistration::registrationErrorFunctionWithFirstAndSecondDerivativeLM(
    const Eigen::Matrix<double, 6, 1>& x, MultiResolutionSurfelRegistration::RegistrationFunctionParameters& params,
    double& f, Eigen::Matrix<double, 6, 1>& df, Eigen::Matrix<double, 6, 6>& d2f,
    MultiResolutionSurfelRegistration::AssociationList& associations)
{
  double sumError = 0.0;
  double sumWeight = 0.0;

  df.setZero();
  d2f.setZero();

  const double tx = x(0);
  const double ty = x(1);
  const double tz = x(2);
  const double qx = x(3);
  const double qy = x(4);
  const double qz = x(5);
  if (qx * qx + qy * qy + qz * qz > 1.0)
    ROS_ERROR_STREAM("quaternion not stable!!");
  const double qw = params.lastWSign * sqrtf(1.0 - qx * qx - qy * qy - qz * qz);  // retrieve sign from last qw

  GradientFunctorLM gf(&associations, params, tx, ty, tz, qx, qy, qz, qw, true);

  pcl::StopWatch timer;
  timer.reset();
  if (PARALLEL)
    tbb::parallel_for(tbb::blocked_range<size_t>(0, associations.size()), gf);
  else
    std::for_each(associations.begin(), associations.end(), gf);

  ROS_DEBUG_STREAM_NAMED("surfel_registration", "GradientFunctorLM took: " << timer.getTime());
  int cidx = 0;
  if (params.correspondences_source_points_)
  {
    params.correspondences_source_points_->points.reserve(10 * associations.size());
    params.correspondences_target_points_->points.reserve(10 * associations.size());
  }

  double numMatches = 0;
  for (MultiResolutionSurfelRegistration::AssociationList::iterator it2 = associations.begin();
       it2 != associations.end(); ++it2)
  {
    //		for( MultiResolutionSurfelRegistration::SingleAssociationList::iterator it = it2->associations_.begin(); it !=
    //it2->associations_.end(); ++it ) {
    for (unsigned int i = 0; i < it2->associations_.size(); i++)
    {
      MultiResolutionSurfelRegistration::SingleAssociation* it = &it2->associations_[i];

      if (!it->match)
        continue;

      float weight = it->weight;
      //		float weight = 1.0 / sqrt(it->error);
      //		float weight = it->weight * exp( -0.5 * (it->z - it->f).squaredNorm() * it->inv_sigma2 );

      const Eigen::Matrix<double, 6, 3> JtW = weight * it->df_dx.transpose() * it->W;

      if (isnan(it->error) || isnan(weight))
        ROS_ERROR_STREAM("not ook");
      else
      {
        df += JtW * (it->z - it->f);
        d2f += JtW * it->df_dx;

        sumError += weight * it->error;
        sumWeight += weight;
        numMatches += 1.0;  // nweight;
      }

      if (params.correspondences_source_points_)
      {
        pcl::PointXYZRGB p1;
        pcl::PointXYZRGB p2;

        Eigen::Vector4f pos1;
        pos1.block<3, 1>(0, 0) = it->model_mean.cast<float>();
        pos1(3) = 1.f;
        Eigen::Vector4f pos2;
        pos2.block<3, 1>(0, 0) = it->scene_mean.cast<float>();
        pos2(3) = 1.f;

        p1.x = pos1(0);
        p1.y = pos1(1);
        p1.z = pos1(2);

        p1.r = weight * 255.f;
        p1.g = 0;
        p1.b = (1.f - weight) * 255.f;

        Eigen::Vector4d pos;
        pos.block<3, 1>(0, 0) = pos2.block<3, 1>(0, 0).cast<double>();
        pos(3, 0) = 1.f;

        const Eigen::Vector4f pos_src = pos2;  // gf.currentTransform * pos;

        p2.x = pos_src[0];
        p2.y = pos_src[1];
        p2.z = pos_src[2];

        p2.r = weight * 255.f;
        p2.g = 0;
        p2.b = (1.f - weight) * 255.f;

        cidx++;

        params.correspondences_source_points_->points.push_back(p1);
        params.correspondences_target_points_->points.push_back(p2);
      }
    }
  }

  if (params.correspondences_source_points_)
  {
    params.correspondences_source_points_->points.resize(cidx);
    params.correspondences_target_points_->points.resize(cidx);
  }

  if (sumWeight <= 1e-10)
  {
    sumError = std::numeric_limits<double>::max();
    return false;
  }
  else if (numMatches < REGISTRATION_MIN_NUM_SURFELS)
  {
    sumError = std::numeric_limits<double>::max();
    ROS_ERROR_STREAM("not enough surfels for robust matching " << numMatches);
    return false;
  }

  //	f = sumError;

  f = sumError / sumWeight;
  df = df / sumWeight;
  d2f = d2f / sumWeight;

  if (use_prior_pose_)
  {
    f += (x - prior_pose_mean_).transpose() * prior_pose_invcov_ * (x - prior_pose_mean_);
    df += prior_pose_invcov_ * (prior_pose_mean_ - x);
    d2f += prior_pose_invcov_;
  }

  ROS_DEBUG_STREAM_NAMED("surfel_registration", "df " << df << " d2f " << d2f << " sumWeight " << sumWeight
                                                      << " sumError " << sumError);

  return true;
}

bool MultiResolutionSurfelRegistration::estimateTransformationLevenbergMarquardt(
    MapType* model, MapType* scene, Eigen::Matrix4d& transform,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr correspondencesSourcePoints,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr correspondencesTargetPoints, int maxIterations)
{
  const double tau = 10e-5;
  const double min_delta = 1e-3;

  last_cov_ = Eigen::Matrix<double, 6, 6>::Zero();

  Eigen::Matrix4d currentTransform = transform;

  // set up the minimization algorithm
  MultiResolutionSurfelRegistration::RegistrationFunctionParameters params;
  params.scene = scene;

  std::vector<mrs_laser_maps::GridCellType*> cells;
  std::vector<Eigen::Vector3f> offsets;
  scene->getOccupiedCellsWithOffset(cells, offsets);

  params.scene_num_points_ = 0.0;
  for (size_t i = 0; i < cells.size(); ++i)
  {
    MultiResolutionSurfelRegistration::CellInfo c;
    c.cell_ = cells[i];
    c.offset_ = offsets[i].cast<double>();
    params.scene_cells.push_back(c);
    //		params.scene_num_points_ += cells[i]->surfel_.num_points_;
  }

  params.scene_num_points_ = cells.size();

  model->getOccupiedCellsWithOffset(cells, offsets);

  params.model = model;
  params.model_num_points_ = model->getNumCellPoints();
  //	params.model_num_points_ = cells.size();
  params.transform = &currentTransform;
  params.correspondences_source_points_ = correspondencesSourcePoints;
  params.correspondences_target_points_ = correspondencesTargetPoints;
  params.prior_prob = prior_prob_;
  params.sigma_size_factor = sigma_size_factor_;
  params.soft_assoc_c1 = soft_assoc_c1_;
  params.soft_assoc_c2 = soft_assoc_c2_;
  params.soft_assoc_c3 = soft_assoc_c3_;

  // initialize with current transform
  Eigen::Matrix<double, 6, 1> x;
  Eigen::Quaterniond q(currentTransform.block<3, 3>(0, 0));

  x(0) = currentTransform(0, 3);
  x(1) = currentTransform(1, 3);
  x(2) = currentTransform(2, 3);
  x(3) = q.x();
  x(4) = q.y();
  x(5) = q.z();
  params.lastWSign = q.w() / fabsf(q.w());

  pcl::StopWatch stopwatch;

  Eigen::Matrix<double, 6, 1> df;
  Eigen::Matrix<double, 6, 6> d2f;

  const Eigen::Matrix<double, 6, 6> id6 = Eigen::Matrix<double, 6, 6>::Identity();
  double mu = -1.0;
  double nu = 2;

  double last_error = std::numeric_limits<double>::max();

  MultiResolutionSurfelRegistration::AssociationList associations;

  bool reassociate = true;

  bool reevaluateGradient = true;

  bool retVal = true;

  int iter = 0;
  while (iter < maxIterations)
  {
    ROS_DEBUG_STREAM_NAMED("surfel_registration", "transform at iteration " << iter << " " << transform);

    double sigma = 0.1;

    if (reevaluateGradient)
    {
      if (!associate_once_ || reassociate)
      {
        stopwatch.reset();
        associations.clear();
        associateMapsBreadthFirstParallel(associations, model, scene, params.scene_cells, transform, sigma);

        ROS_DEBUG_STREAM_NAMED("surfel_registration", "associations: " << associations.size()
                                                                       << " took: " << stopwatch.getTime() << " ms");
      }

      stopwatch.reset();
      retVal = registrationErrorFunctionWithFirstAndSecondDerivativeLM(x, params, last_error, df, d2f, associations);
      ROS_DEBUG_STREAM_NAMED("surfel_registration", "registrationErrorFunctionWithFirstAndSecondDerivativeLM took: "
                                                        << stopwatch.getTime() << " ms");
    }

    reevaluateGradient = false;

    if (!retVal)
    {
      ROS_ERROR("registration failed");
      return false;
    }

    if (mu < 0)
    {
      mu = tau * std::max(d2f.maxCoeff(), -d2f.minCoeff());
    }

    Eigen::Matrix<double, 6, 1> delta_x = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 6> d2f_inv = Eigen::Matrix<double, 6, 6>::Zero();
    if (fabsf(d2f.determinant()) > std::numeric_limits<double>::epsilon())
    {
      d2f_inv = (d2f + mu * id6).inverse();

      delta_x = d2f_inv * df;
    }

    if (delta_x.norm() < min_delta)
    {
      ROS_DEBUG_STREAM_NAMED("surfel_registration", "delta_x.norm() < min_delta at iteration " << iter << " "
                                                                                               << delta_x);
      ROS_DEBUG_STREAM_NAMED("surfel_registration", "d2f_inv " << d2f_inv << " df " << df);

      if (reassociate)
      {
        last_cov_ = d2f.inverse();
        break;
      }

      reassociate = true;
      reevaluateGradient = true;
    }
    else
      reassociate = false;

    double qx = x(3);
    double qy = x(4);
    double qz = x(5);
    double qw = params.lastWSign * sqrt(1.0 - qx * qx - qy * qy - qz * qz);

    currentTransform.setIdentity();
    currentTransform.block<3, 3>(0, 0) = Eigen::Matrix3d(Eigen::Quaterniond(qw, qx, qy, qz));
    currentTransform(0, 3) = x(0);
    currentTransform(1, 3) = x(1);
    currentTransform(2, 3) = x(2);

    qx = delta_x(3);
    qy = delta_x(4);
    qz = delta_x(5);
    qw = sqrt(1.0 - qx * qx - qy * qy - qz * qz);

    Eigen::Matrix4d deltaTransform = Eigen::Matrix4d::Identity();
    deltaTransform.block<3, 3>(0, 0) = Eigen::Matrix3d(Eigen::Quaterniond(qw, qx, qy, qz));
    deltaTransform(0, 3) = delta_x(0);
    deltaTransform(1, 3) = delta_x(1);
    deltaTransform(2, 3) = delta_x(2);

    Eigen::Matrix4d newTransform = deltaTransform * currentTransform;

    Eigen::Matrix<double, 6, 1> x_new;
    x_new(0) = newTransform(0, 3);
    x_new(1) = newTransform(1, 3);
    x_new(2) = newTransform(2, 3);

    Eigen::Quaterniond q_new(newTransform.block<3, 3>(0, 0));
    x_new(3) = q_new.x();
    x_new(4) = q_new.y();
    x_new(5) = q_new.z();

    //		MultiResolutionSurfelRegistration::AssociationList newAssociations;
    //		associateMapsBreadthFirstParallel( newAssociations, model, scene, newTransform, sigma );
    double new_error = 0.0;
    stopwatch.reset();
    bool retVal2 = registrationErrorFunctionLM(x_new, params, new_error, associations);

    ROS_DEBUG_STREAM_NAMED("surfel_registration", "registrationErrorFunctionLM took : " << stopwatch.getTime()
                                                                                        << " last_error: " << last_error
                                                                                        << " new_error: " << new_error);

    if (!retVal2)
      return false;

    double rho = (last_error - new_error) / (delta_x.transpose() * (mu * delta_x + df));

    if (rho > 0)
    {
      x = x_new;

      last_cov_ = d2f.inverse();

      mu *= std::max(0.333, 1.0 - pow(2.0 * rho - 1.0, 3.0));
      nu = 2;

      reevaluateGradient = true;
    }
    else
    {
      mu *= nu;
      nu *= 2.0;
    }

    qx = x(3);
    qy = x(4);
    qz = x(5);
    qw = params.lastWSign * sqrt(1.0 - qx * qx - qy * qy - qz * qz);

    if (isnan(qw) || fabsf(qx) > 1.f || fabsf(qy) > 1.f || fabsf(qz) > 1.f)
    {
      return false;
    }

    transform.setIdentity();
    transform.block<3, 3>(0, 0) = Eigen::Matrix3d(Eigen::Quaterniond(qw, qx, qy, qz));
    transform(0, 3) = x(0);
    transform(1, 3) = x(1);
    transform(2, 3) = x(2);

    //		last_error = new_error;

    iter++;
  }

  return retVal;
}

bool MultiResolutionSurfelRegistration::estimatePoseCovariance(Eigen::Matrix<double, 6, 6>& poseCov, MapType* model,
                                                               MapType* scene, Eigen::Matrix4d& transform)
{
  poseCov = last_cov_;
  return true;
}

Eigen::Matrix<double, 6, 1> MultiResolutionSurfelRegistration::transform2Pose(const Eigen::Matrix4d& transform,
                                                                              double& qw_sign)
{
  Eigen::Matrix<double, 6, 1> pose;
  pose.block<3, 1>(0, 0) = transform.block<3, 1>(0, 3);
  Eigen::Quaterniond q(transform.block<3, 3>(0, 0));
  pose(3) = q.x();
  pose(4) = q.y();
  pose(5) = q.z();
  if (q.w() != 0.0)
    qw_sign = q.w() / fabsf(q.w());
  else
    qw_sign = 1.0;
  return pose;
}

Eigen::Matrix4d MultiResolutionSurfelRegistration::pose2Transform(const Eigen::Matrix<double, 6, 1>& pose,
                                                                  double qw_sign)
{
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  double qw = qw_sign * sqrt(1.0 - pose.block<3, 1>(3, 0).squaredNorm());
  Eigen::Quaterniond q(pose(3), pose(4), pose(5), qw);

  transform.block<3, 3>(0, 0) = q.matrix();
  transform.block<3, 1>(0, 3) = pose.block<3, 1>(0, 0);

  return transform;
}

bool MultiResolutionSurfelRegistration::estimatePoseCovarianceUnscented(Eigen::Matrix<double, 6, 6>& poseCov,
                                                                        MapType* model, MapType* scene,
                                                                        Eigen::Matrix4d& transform)
{
  poseCov.setZero();

  const double h = 0.1;

  double qw_sign = 1;
  Eigen::Matrix<double, 6, 1> pose = transform2Pose(transform, qw_sign);

  // set up the minimization algorithm
  MultiResolutionSurfelRegistration::RegistrationFunctionParameters params;
  params.scene = scene;

  std::vector<mrs_laser_maps::GridCellType*> cells;
  std::vector<Eigen::Vector3f> offsets;
  scene->getOccupiedCellsWithOffset(cells, offsets);

  params.scene_num_points_ = 0.0;
  for (size_t i = 0; i < cells.size(); ++i)
  {
    MultiResolutionSurfelRegistration::CellInfo c;
    c.cell_ = cells[i];
    c.offset_ = offsets[i].cast<double>();
    params.scene_cells.push_back(c);
  }

  params.scene_num_points_ = cells.size();

  model->getOccupiedCellsWithOffset(cells, offsets);

  params.model = model;
  params.model_num_points_ = model->getNumCellPoints();
  params.transform = &transform;
  params.prior_prob = prior_prob_;
  params.lastWSign = qw_sign;

  MultiResolutionSurfelRegistration::AssociationList associations;
  double error = 0.0;
  associations.clear();
  associateMapsBreadthFirstParallel(associations, model, scene, params.scene_cells, transform, 0.1);
  registrationErrorFunctionLM(pose, params, error, associations);

  for (unsigned int i = 0; i < 3; i++)
  {
    for (unsigned int j = 0; j <= i; j++)
    {
      if (i == j)
      {
        Eigen::Matrix<double, 6, 1> x_p2i = pose;
        x_p2i(i) += 2 * h;
        double error_p2i = 0.0;
        associations.clear();
        Eigen::Matrix4d T_p2i = pose2Transform(x_p2i, qw_sign);
        associateMapsBreadthFirstParallel(associations, model, scene, params.scene_cells, T_p2i, 0.1);
        registrationErrorFunctionLM(x_p2i, params, error_p2i, associations);

        Eigen::Matrix<double, 6, 1> x_pi = pose;
        x_pi(i) += h;
        double error_pi = 0.0;
        associations.clear();
        Eigen::Matrix4d T_pi = pose2Transform(x_pi, qw_sign);
        associateMapsBreadthFirstParallel(associations, model, scene, params.scene_cells, T_pi, 0.1);
        registrationErrorFunctionLM(x_pi, params, error_pi, associations);

        Eigen::Matrix<double, 6, 1> x_m2i = pose;
        x_m2i(i) -= 2 * h;
        double error_m2i = 0.0;
        associations.clear();
        Eigen::Matrix4d T_m2i = pose2Transform(x_m2i, qw_sign);
        associateMapsBreadthFirstParallel(associations, model, scene, params.scene_cells, T_m2i, 0.1);
        registrationErrorFunctionLM(x_m2i, params, error_m2i, associations);

        Eigen::Matrix<double, 6, 1> x_mi = pose;
        x_mi(i) -= h;
        double error_mi = 0.0;
        associations.clear();
        Eigen::Matrix4d T_mi = pose2Transform(x_mi, qw_sign);
        associateMapsBreadthFirstParallel(associations, model, scene, params.scene_cells, T_mi, 0.1);
        registrationErrorFunctionLM(x_mi, params, error_mi, associations);

        //				error_p2i = std::max( error, error_p2i );
        //				error_pi = std::max( error, error_pi );
        //				error_mi = std::max( error, error_mi );
        //				error_m2i = std::max( error, error_m2i );

        poseCov(i, j) = (-error_p2i + 16.0 * error_pi - 30.0 * error + 16.0 * error_mi - error_m2i) / (12.0 * h * h);
      }
      else
      {
        Eigen::Matrix<double, 6, 1> x_pipj = pose;
        x_pipj(i) += h;
        x_pipj(j) += h;
        double error_pipj = 0.0;
        associations.clear();
        Eigen::Matrix4d T_pipj = pose2Transform(x_pipj, qw_sign);
        associateMapsBreadthFirstParallel(associations, model, scene, params.scene_cells, T_pipj, 0.1);
        registrationErrorFunctionLM(x_pipj, params, error_pipj, associations);

        Eigen::Matrix<double, 6, 1> x_pimj = pose;
        x_pimj(i) += h;
        x_pimj(j) -= h;
        double error_pimj = 0.0;
        associations.clear();
        Eigen::Matrix4d T_pimj = pose2Transform(x_pimj, qw_sign);
        associateMapsBreadthFirstParallel(associations, model, scene, params.scene_cells, T_pimj, 0.1);
        registrationErrorFunctionLM(x_pimj, params, error_pimj, associations);

        Eigen::Matrix<double, 6, 1> x_mipj = pose;
        x_mipj(i) -= h;
        x_mipj(j) += h;
        double error_mipj = 0.0;
        associations.clear();
        Eigen::Matrix4d T_mipj = pose2Transform(x_mipj, qw_sign);
        associateMapsBreadthFirstParallel(associations, model, scene, params.scene_cells, T_mipj, 0.1);
        registrationErrorFunctionLM(x_mipj, params, error_mipj, associations);

        Eigen::Matrix<double, 6, 1> x_mimj = pose;
        x_mimj(i) -= h;
        x_mimj(j) -= h;
        double error_mimj = 0.0;
        associations.clear();
        Eigen::Matrix4d T_mimj = pose2Transform(x_mimj, qw_sign);
        associateMapsBreadthFirstParallel(associations, model, scene, params.scene_cells, T_mimj, 0.1);
        registrationErrorFunctionLM(x_mimj, params, error_mimj, associations);

        //				error_pipj = std::max( error, error_pipj );
        //				error_pimj = std::max( error, error_pimj );
        //				error_mipj = std::max( error, error_mipj );
        //				error_mimj = std::max( error, error_mimj );

        poseCov(i, j) = poseCov(j, i) = (error_pipj - error_pimj - error_mipj + error_mimj) / (4.0 * h * h);
      }
    }
  }

  poseCov.block<3, 3>(0, 0) = poseCov.block<3, 3>(0, 0).inverse().eval();

  //	poseCov = poseCov.inverse().eval();

  return true;
}
