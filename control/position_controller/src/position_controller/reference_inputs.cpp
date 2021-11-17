/**
 *  @file   reference_inputs.cpp
 *  @brief  quadrotor position control's reference inputs related functionality implementation
 *  @author thor
 *  @date   17.11.2021
 */
#include "position_controller/reference_inputs.h"

namespace position_controller {

/**
 *  @detail
 */
ReferenceInputs::ReferenceInputs(
    const quadrotor_common::QuadrotorStateEstimate& state_est,
    const quadrotor_common::QuadrotorTrajectoryPoint& state_ref)
    : state_estimate(state_est),
      reference_state(state_ref) {
  //  constraints based on reference heading i.e.,
  //  projection of x_B into x_W - y_W plane should be collinear to x_C
  const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
      Eigen::AngleAxisd(reference_state.heading, Eigen::Vector3d::UnitZ()));
  x_C = q_heading * Eigen::Vector3d::UnitX();
  y_C = q_heading * Eigen::Vector3d::UnitY();

}

/**
 *  @detail
 */
void ReferenceInputs::computeReferenceOrientation() {
  x_B = computeRobustBodyXAxis();
  y_B = computeRobustBodyYAxis();
  z_B = x_B.cross(y_B);

  const Eigen::Matrix3d R_W_B((Eigen::Matrix3d() << x_B, y_B, z_B).finished());
  orientation = Eigen::Quaterniond(R_W_B);
}

/**
 *  @detail For y_C = [-sin(phi) cos(phi) 0]^T, alpha = v_dot + g.z_W + dx.v
 *          Check if norm(y_C x alpha) == 0
 *            if true, check if norm(x_B_est - (x_B_est^T.y_C)y_C) == 0
 *                      if true, set x_B = x_C
 *                      else, set x_B = normalized(x_B_est - (x_B_est^T.y_C)y_C)
 *            else, set x_B = normalized(y_C x alpha)
 */
Eigen::Vector3d ReferenceInputs::computeRobustBodyXAxis() const {
  const Eigen::Vector3d alpha = reference_state.acceleration - kGravity_ + \
      dx * reference_state.velocity;
  Eigen::Vector3d x_B = y_C.cross(alpha);

  //  check if y_C is collinear to alpha or alpha = 0
  if (quadrotor_common::isAlmostZero(x_B.norm(), kAlmostZeroValueThreshold_)) {
    //  project x_B estimate into x_C - z_C plane, using scalar projection onto y_C
    //  followed by vector rejection
    const Eigen::Vector3d x_B_est = state_estimate.orientation * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d x_B_proj = x_B_est - (x_B_est.dot(y_C))*y_C;

    //  check if norm(x_B_proj) = 0
    if (quadrotor_common::isAlmostZero(x_B_proj.norm(), kAlmostZeroValueThreshold_)) {
      x_B = x_C;
    } //  special case which may lead to jumps in the desired orientation
    else {
      x_B = x_B_proj.normalized();
    }
  } //  handle singularity case
  else {
    x_B.normalize();
  } //  normalize

  return x_B;
}

Eigen::Vector3d ReferenceInputs::computeRobustBodyYAxis() const {
  const Eigen::Vector3d beta = reference_state.acceleration - kGravity_ + \
      dy * reference_state.velocity;
  Eigen::Vector3d y_B = beta.cross(x_B);

  //  check if x_B is collinear to beta or beta = 0
  if (quadrotor_common::isAlmostZero(y_B.norm(), kAlmostZeroValueThreshold_)) {
    //  y_B should also be perpendicular to z_B_est and x_B
    const Eigen::Vector3d z_B_est = state_estimate.orientation * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d y_B_temp = z_B_est.cross(x_B);

    //  check if norm(y_B_temp) = 0
    if (quadrotor_common::isAlmostZero(y_B_temp.norm(), kAlmostZeroValueThreshold_)) {
      y_B = y_C;
    } //  special case which may lead to jumps in the desired orientation
    else {
      y_B = y_B_temp.normalized();
    }
  } //  handle singularity case
  else {
    y_B.normalize();
  } //  normalize
}

} /*  namespace position_controller  */
