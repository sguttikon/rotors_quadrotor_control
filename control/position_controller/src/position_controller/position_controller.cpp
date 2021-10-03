/**
 *  @file   position_controller.cpp
 *  @brief  quadrotor's position control related functionality implementation
 *  @author neo
 *  @data   03.10.2021
 */
#include "position_controller/position_controller.h"

// 3rd party dependencies
#include <Eigen/Dense>

// quadrotor_common dependencies
#include "quadrotor_common/quadrotor_control_command.h"
#include "quadrotor_common/quadrotor_state_estimate.h"
#include "quadrotor_common/quadrotor_trajectory_point.h"

namespace position_controller {

/**
 *  @detail PositionController's default constructor definition
 */
PositionController::PositionController() {}

/**
 *  @detail PositionController's default destructor definition
 */
PositionController::~PositionController() {}

/**
 *
 */
quadrotor_common::QuadrotorControlCommand PositionController::run(
    const quadrotor_common::QuadrotorStateEstimate& state_estimate,
    const quadrotor_common::QuadrotorTrajectoryPoint& reference_state) {

  // compute reference inputs as feed forward terms

}

/**
 *
 */
quadrotor_common::QuadrotorControlCommand PositionController::computeNominalReferenceInputs(
    const quadrotor_common::QuadrotorTrajectoryPoint& reference_state,
    const Eigen::Quaterniond& attitude_estimate) const {

  // constraints based on acceleration equation from quadrotor's dynamics
  //  i.e x_B*alpha = 0, y_B*beta = 0 and z_b*gamma = collective_thrust
  const Eigen::Vector3d alpha = reference_state.acceleration - kGravity_;
  const Eigen::Vector3d beta = reference_state.acceleration - kGravity_;
  const Eigen::Vector3d gamma = reference_state.acceleration - kGravity_;

  // projection of x_B into x_W - y_W plane will be collinear with x_C
  const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
      Eigen::AngleAxisd(reference_state.heading, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();

  // 1. compute reference orientation input R
  const Eigen::Vector3d x_B = computeRobustBodyXAxis(x_C, y_C, alpha, attitude_estimate);
}

/**
 *
 */
Eigen::Vector3d PositionController::computeRobustBodyXAxis(
    const Eigen::Vector3d& x_C,
    const Eigen::Vector3d& y_C,
    const Eigen::Vector3d& alpha,
    const Eigen::Quaterniond& attitude_estimate) const {

  Eigen::Vector3d x_B = y_C.cross(alpha);

  // check if y_C is collinear to alpha
  if (isAlmostZero(x_B.norm())) {

    // project estimated x_B into x_C - z_C plane using scalar projection onto y_C
    const Eigen::Vector3d x_B_est = attitude_estimate * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d x_B_proj = x_B_est - (x_B_est.dot(y_C)) * y_C;
    if (isAlmostZero(x_B_proj.norm()))
      x_B = x_C;
    else
      x_B = x_B_proj.normalized();

  } // handle singuarity case
  else {
    x_B.normalize();
  } // normalize

  return x_B;
}

/**
 *
 */
bool PositionController::isAlmostZero(const double value) const {
  return fabs(value) < kAlmostZeroValueThreshold_;
}

} /* namespace position_controller */
