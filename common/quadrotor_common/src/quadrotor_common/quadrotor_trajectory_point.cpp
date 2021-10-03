/**
 *  @file   quadrotor_trajectory_point.cpp
 *  @brief  quadrotor's state along trajectory related functionality implementation
 *  @author neo
 *  @data   03.10.2021
 */
#include "quadrotor_common/quadrotor_trajectory_point.h"

// 3rd party dependencies
#include <Eigen/Dense>

namespace quadrotor_common {

/**
 *  @detail QuadrotorTrajectoryPoint's default constructor definition.
 */
QuadrotorTrajectoryPoint::QuadrotorTrajectoryPoint()
    : position(Eigen::Vector3d::Zero()),
      orientation(Eigen::Quaterniond::Identity()),
      heading(0.0),
      velocity(Eigen::Vector3d::Zero()),
      acceleration(Eigen::Vector3d::Zero()),
      jerk(Eigen::Vector3d::Zero()),
      snap(Eigen::Vector3d::Zero()),
      bodyrates(Eigen::Vector3d::Zero()),
      angular_acceleration(Eigen::Vector3d::Zero()),
      angular_jerk(Eigen::Vector3d::Zero()),
      angular_snap(Eigen::Vector3d::Zero()),
      heading_rate(0.0),
      heading_acceleration(0.0) {}

/**
 *  @detail QuadrotorTrajectoryPoint's default destructor definition.
 */
QuadrotorTrajectoryPoint::~QuadrotorTrajectoryPoint() {}

} /* namespace quadrotor_common */
