#pragma once

#include <ceres/ceres.h>
#include <ros/assert.h>

#include <Eigen/Dense>

#include "../parameters.h"
#include "../utility/tic_toc.h"
#include "../utility/utility.h"

/// 2:残差的长度(err_x,err_y)
/// 7:第一个优化参数的长度para_Pose[imu_i]
/// 7:第二个优化参数的长度para_Pose[imu_j]
/// 7:第3个优化参数的长度para_Ex_Pose[0]
/// 1:第4个优化参数feature_inverse_depth的长度para_Feature[feature_index]
class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1> {
 public:
  ProjectionFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j);
  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
  void check(double **parameters);

  Eigen::Vector3d pts_i, pts_j;
  Eigen::Matrix<double, 2, 3> tangent_base;
  static Eigen::Matrix2d sqrt_info;
  static double sum_t;
};
