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
 *  @detail c = z_B^T(v_dot + g.z_W + dz.v)
 */
void ReferenceInputs::computeReferenceCollectiveThrust() {
  const Eigen::Vector3d gamma = reference_state.acceleration - kGravity_ + \
      dz * reference_state.velocity;
  c = z_B.dot(gamma);
}

/**
 *  @detail
 */
void ReferenceInputs::computeReferenceBodyrates() {
  const double B1 = c - (dz - dx)*(z_B.dot(reference_state.velocity));
  const double C1 = -(dx - dy)*(y_B.dot(reference_state.velocity));
  const double D1 = x_B.dot(reference_state.jerk) + \
      dx * x_B.dot(reference_state.acceleration);
  const double A2 = c + (dy - dz)*(z_B.dot(reference_state.velocity));
  const double C2 = (dx - dy)*(x_B.dot(reference_state.velocity));
  const double D2 = -y_B.dot(reference_state.jerk) - \
      dy * y_B.dot(reference_state.acceleration);
  const double B3 = -y_C.dot(z_B);
  const double C3 = (y_C.cross(z_B)).norm();
  const double D3 = reference_state.heading_rate * x_C.dot(x_B);

  // check if B1*C3 - B3*C1 is 0
  const double denominator = B1*C3 - B3*C1;
  if (quadrotor_common::isAlmostZero(denominator, kAlmostZeroValueThreshold_)) {
    bodyrates = Eigen::Vector3d::Zero();
  } //  zero bodyrates
  else{
    if (quadrotor_common::isAlmostZero(A2, kAlmostZeroValueThreshold_)) {
      bodyrates.x() = 0.0;
    } //  zero bodyrates.x()
    else {
      bodyrates.x() = (-B1*C2*D3 + B1*C3*D2 - B3*C1*D2 + B3*C2*D1)/ \
          (A2*denominator);
    }
    bodyrates.y() = (-C1*D3 + C3*D1)/denominator;
    bodyrates.z() = ( B1*D3 - B3*D1)/denominator;
  } //  compute bodyrates
}

/**
 *  @detail
 */
void ReferenceInputs::computeReferenceAngularAccelerations() {
  const Eigen::Matrix3d omega_hat = quadrotor_common::skew(bodyrates);
  const Eigen::Matrix3d R((Eigen::Matrix3d() << x_B, y_B, z_B).finished());
  const Eigen::Matrix3d D = Eigen::Vector3d(dx, dy, dz).asDiagonal();
  const double c_dot = z_B.dot(reference_state.jerk) + \
      bodyrates.x()*(dy - dz)*(y_B.dot(reference_state.velocity)) + \
      bodyrates.y()*(dz - dx)*(x_B.dot(reference_state.velocity)) + \
      dz * z_B.dot(reference_state.acceleration);
  const Eigen::Vector3d xi =
      R * (omega_hat * omega_hat * D + D * omega_hat * omega_hat +
        2 * omega_hat * D * omega_hat.transpose()
      ) * R.transpose() * reference_state.velocity + \
      2 * R * (omega_hat * D + D * omega_hat.transpose()
      ) * R.transpose() * reference_state.acceleration + \
      R * D * R.transpose() * reference_state.jerk;

  const double B1 = c - (dz - dx)*(z_B.dot(reference_state.velocity));
  const double C1 = -(dx - dy)*(y_B.dot(reference_state.velocity));
  const double E1 = x_B.dot(reference_state.snap) - 2*c_dot * bodyrates.y() - \
      c * bodyrates.x() * bodyrates.z() + x_B.dot(xi);
  const double A2 = c + (dy - dz)*(z_B.dot(reference_state.velocity));
  const double C2 = (dx - dy)*(x_B.dot(reference_state.velocity));
  const double E2 = -y_B.dot(reference_state.snap) - 2*c_dot * bodyrates.x() + \
      c * bodyrates.y() * bodyrates.z() - y_B.dot(xi);
  const double B3 = -y_C.dot(z_B);
  const double C3 = (y_C.cross(z_B)).norm();
  const double E3 = reference_state.heading_acceleration * x_C.dot(x_B) + \
      2*reference_state.heading_rate * bodyrates.z() * x_C.dot(y_B) - \
      2*reference_state.heading_rate * bodyrates.y() * x_C.dot(z_B) - \
      bodyrates.x() * bodyrates.y() * y_C.dot(y_B) - \
      bodyrates.x() * bodyrates.z() * y_C.dot(z_B);

  // check if B1*C3 - B3*C1 is 0
  const double denominator = B1*C3 - B3*C1;
  if (quadrotor_common::isAlmostZero(denominator, kAlmostZeroValueThreshold_)) {
    angular_accelerations = Eigen::Vector3d::Zero();
  } //  zero angular accelerations
  else{
    if (quadrotor_common::isAlmostZero(A2, kAlmostZeroValueThreshold_)) {
      angular_accelerations.x() = 0.0;
    } //  zero angular_accelerations.x()
    else {
      angular_accelerations.x() = (-B1*C2*E3 + B1*C3*E2 - B3*C1*E2 + B3*C2*E1)/ \
          (A2*denominator);
    }
    angular_accelerations.y() = (-C1*E3 + C3*E1)/denominator;
    angular_accelerations.z() = ( B1*E3 - B3*E1)/denominator;
  } //  compute angular accelerations
}

/**
 *  @detail For x_C = [cos(phi) sin(phi) 0]^T, y_C = [-sin(phi) cos(phi) 0]^T,
 *          and alpha = v_dot + g.z_W + dx.v
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

  //  check if y_C is collinear to alpha or alpha is 0
  if (quadrotor_common::isAlmostZero(x_B.norm(), kAlmostZeroValueThreshold_)) {
    //  project x_B estimate into x_C - z_C plane, using scalar projection onto y_C
    //  followed by vector rejection
    const Eigen::Vector3d x_B_est = state_estimate.orientation * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d x_B_proj = x_B_est - (x_B_est.dot(y_C))*y_C;

    //  check if norm(x_B_proj) is 0
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

/**
 *  @detail For x_C = [cos(phi) sin(phi) 0]^T, y_C = [-sin(phi) cos(phi) 0]^T,
 *          and beta = v_dot + g.z_W + dy.v
 *          Check if norm(beta x x_B) == 0
 *            if true, check if norm(z_B_est x x_B) == 0
 *                      if true, set y_B = y_C
 *                      else, set y_B = normalized(z_B_est x x_B)
 *            else, set y_B = normalized(beta x x_B)
 */
Eigen::Vector3d ReferenceInputs::computeRobustBodyYAxis() const {
  const Eigen::Vector3d beta = reference_state.acceleration - kGravity_ + \
      dy * reference_state.velocity;
  Eigen::Vector3d y_B = beta.cross(x_B);

  //  check if x_B is collinear to beta or beta is 0
  if (quadrotor_common::isAlmostZero(y_B.norm(), kAlmostZeroValueThreshold_)) {
    //  y_B should also be perpendicular to z_B_est and x_B
    const Eigen::Vector3d z_B_est = state_estimate.orientation * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d y_B_temp = z_B_est.cross(x_B);

    //  check if norm(y_B_temp) is 0
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
