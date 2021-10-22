/**
 *  @file   reference_inputs.cpp
 *  @brief  quadrotor position control's reference inputs related functionality implementation
 *  @author neo
 *  @data   16.10.2021
 */
#include "reference_inputs/reference_inputs.h"

namespace position_controller {

/**
 *  @detail Perform the following:
 *          check if norm(y_C x alpha) == 0
 *            if true, the check if norm(x_B_est - (x_B_est^T y_C)y_C) == 0
 *                      if true, set x_B = x_C
 *                      else, set x_B = normalized(x_B_est - (x_B_est^T y_C)y_C)
 *            else, set x_B = normalized(y_C x alpha)
 */
Eigen::Vector3d ReferenceInputs::computeRobustBodyXAxis(
    const Eigen::Vector3d& y_C,
    const Eigen::Vector3d& alpha,
    const Eigen::Quaterniond& attitude_estimate,
    const Eigen::Vector3d& x_C) const {

  Eigen::Vector3d x_B = y_C.cross(alpha);

  // check if y_C is collinear to alpha
  if (isAlmostZero(x_B.norm())) {

    // project estimated x_B into x_C - z_C plane using scalar projection onto y_C, followed by vector rejection
    const Eigen::Vector3d x_B_est = attitude_estimate * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d x_B_proj = x_B_est - (x_B_est.dot(y_C)) * y_C;
    if (isAlmostZero(x_B_proj.norm()))
      x_B = x_C;  // special case which may lead to jumps in the desired orientation
    else
      x_B = x_B_proj.normalized();

  } // handle singuarity case
  else {
    x_B.normalize();
  } // normalize

  return x_B;
}

/**
 *  @detail Perform the following:
 *          check if norm(beta x x_B) == 0
 *            if true, check if norm(z_B_est x x_B) == 0
 *                      if true, set y_B = y_C
 *                      else, set y_B = normalized(z_B_est x x_B)
 *            else, set y_B = normalized(beta x x_B)
 */
Eigen::Vector3d ReferenceInputs::computeRobustBodyYAxis(
    const Eigen::Vector3d& x_B,
    const Eigen::Vector3d& beta,
    const Eigen::Quaterniond& attitude_estimate,
    const Eigen::Vector3d& y_C) const {

  Eigen::Vector3d y_B = beta.cross(x_B);

  // check if x_B is collinear to beta
  if (isAlmostZero(y_B.norm())) {

    // project estimated x_B into x_C - z_C plane using scalar projection onto y_C
    const Eigen::Vector3d z_B_est = attitude_estimate * Eigen::Vector3d::UnitZ();
    y_B = z_B_est.cross(x_B);
    if (isAlmostZero(y_B.norm()))
      y_B = y_C;  // special case which may lead to jumps in the desired orientation
    else
      y_B.normalize();

  } // handle singuarity case
  else {
    y_B.normalize();
  } // normalize

  return y_B;
}

/**
 *
 */
bool ReferenceInputs::isAlmostZero(const double value) const {
  return fabs(value) < kAlmostZeroValueThreshold_;
}

} /* namespace position_controller */
