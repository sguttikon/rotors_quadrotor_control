/**
 *  @file   quadrotor_control_command.cpp
 *  @brief  quadrotor's desired state as control command related functionality implementation
 *  @author neo
 *  @data   03.10.2021
 */
#include "quadrotor_common/quadrotor_control_command.h"

// 3rd party dependencies
#include <Eigen/Dense>
#include <ros/time.h>

namespace quadrotor_common {

/**
 *  @detail QuadrotorControlCommand's default constructor definition.
 */
QuadrotorControlCommand::QuadrotorControlCommand()
    : timestamp(ros::Time::now()),
      control_mode(ControlMode::kNone),
      orientation(Eigen::Quaterniond::Identity()),
      bodyrates(Eigen::Vector3d::Zero()),
      angular_acceleration(Eigen::Vector3d::Zero()),
      collective_thrust(0.0) {}

/**
 *  @detail QuadrotorControlCommand's default destructor definition.
 */
QuadrotorControlCommand::~QuadrotorControlCommand() {}

} /* namespace quadrotor_common */
