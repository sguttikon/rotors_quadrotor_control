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
 *  @detail We use the following Nominal Quadrotor dynamics (where gravity = +9.81):
 *          position_dot  = velocity
 *          velocity_dot  = -gravity * z_W + collective_thrust * z_B
 *          R_dot         = R * bodyrates_hat
 *          bodyrates_dot = inertia_inverse * (torque_inputs - bodyrates x inertia*bodyrates)
 */
 quadrotor_common::QuadrotorControlCommand PositionController::computeReferenceInputs(
     const quadrotor_common::QuadrotorStateEstimate& state_estimate,
     const quadrotor_common::QuadrotorTrajectoryPoint& reference_state) const {

  // // constraints based on reference acceleration using quadrotor's dynamics
  // const Eigen::Vector3d acceleration = reference_state.acceleration - kGravity_;
  //
  // // constraints based on reference heading using
  // // projection of x_B into x_W - y_W plane will be collinear with x_C
  // const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
  //     Eigen::AngleAxisd(reference_state.heading, Eigen::Vector3d::UnitZ()));
  // const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
  // const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();
  //
  // // 1. compute quadrotor's reference orientation input i.e. R_ref = [x_B y_B z_B]
  // const Eigen::Vector3d x_B = computeRobustBodyXAxis(y_C, acceleration, attitude_estimate, x_C);
  // const Eigen::Vector3d y_B = computeRobustBodyYAxis(x_B, acceleration, attitude_estimate, y_C);
  // const Eigen::Vector3d z_B = x_B.cross(y_B);
  // const Eigen::Matrix3d R_W_B((Eigen::Matrix3d() << x_B, y_B, z_B).finished());
  // const Eigen::Quaterniond q_W_B = Eigen::Quaterniond(R_W_B);
  //
  // // 2. compute quadrotor's reference collective thrust input i.e. c_ref
  // const double c = z_B.dot(acceleration);
}

} /* namespace position_controller */
