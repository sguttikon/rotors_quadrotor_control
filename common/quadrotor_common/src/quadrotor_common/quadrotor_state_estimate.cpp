/**
 *  @file   quadrotor_state_estimate.cpp
 *  @brief  quadrotor's state estimation related functionality implementation
 *  @author neo
 *  @data   03.10.2021
 */
#include "quadrotor_common/quadrotor_state_estimate.h"

// 3rd party dependencies
#include <Eigen/Dense>
#include <ros/time.h>

namespace quadrotor_common {

/**
 *  @detail QuadrotorStateEstimate's default constructor definition
 */
QuadrotorStateEstimate::QuadrotorStateEstimate()
    : timestamp(ros::Time::now()),
      coordinate_frame(CoordinateFrame::kInvalid),
      position(Eigen::Vector3d::Zero()),
      orientation(Eigen::Quaterniond::Identity()),
      velocity(Eigen::Vector3d::Zero()),
      bodyrates(Eigen::Vector3d::Zero()) {}

/**
 *  @detail QuadrotorStateEstimate's default destructor definition
 */
QuadrotorStateEstimate::~QuadrotorStateEstimate() {}

} /* namespace quadrotor_common */
