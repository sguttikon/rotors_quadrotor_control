/**
 *  @file   reference_inputs.cpp
 *  @brief  quadrotor position control's nominal dynamics reference inputs related functionality implementation
 *  @author neo
 *  @data   16.10.2021
 */
#include "reference_inputs/nominal_reference_inputs.h"

namespace position_controller {

/**
 *  @detail In nominal dynamics scenario, constraints: alpha, beta and gamma
 *          values are equal i.e. to desired acceleration.
 */
Eigen::Quaterniond NominalReferenceInputs::computeDesiredAttitude() const {
  // Constraints based on acceleration i.e. alpha, beta and gamma
  const Eigen::Vector3d desired_acceleration = reference_state.acceleration \
      - kGravity_;

  // Compute robust x_B, y_B and z_B that statisfies all required constraints
  const Eigen::Vector3d x_B = computeRobustBodyXAxis(
      y_C, desired_acceleration, state_estimate.orientation, x_C);
  const Eigen::Vector3d y_B = computeRobustBodyYAxis(
      x_B, desired_acceleration, state_estimate.orientation, y_C);
  const Eigen::Vector3d z_B = x_B.cross(y_B);

  // Construct desired(reference) attitude
  const Eigen::Matrix3d R_W_B((Eigen::Matrix3d() << x_B, y_B, z_B).finished());
  const Eigen::Quaterniond q_W_B = Eigen::Quaterniond(R_W_B);

  return q_W_B;
}

/**
 *  @detail
 */
float NominalReferenceInputs::computeDesiredCollectiveThrust(
    const Eigen::Quaterniond& q_W_B) const {
  // Constraints based on acceleration i.e. alpha, beta and gamma
  const Eigen::Vector3d desired_acceleration = reference_state.acceleration \
      - kGravity_;
  const Eigen::Vector3d z_B = q_W_B * Eigen::Vector3d::UnitZ();

  const float c = z_B.dot(desired_acceleration);
  return c;
}

} /* namespace position_controller */
