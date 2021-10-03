/**
 *  @file   quadrotor_control_command.h
 *  @brief  quadrotor's desired state as control command related functionality declaration & definition
 *  @author neo
 *  @data   03.10.2021
 */
#ifndef QUADROTOR_COMMON_QUADROTOR_CONTROL_COMMAND_H
#define QUADROTOR_COMMON_QUADROTOR_CONTROL_COMMAND_H

// 3rd party dependencies
#include <Eigen/Dense>

namespace quadrotor_common {

/**
 *  @brief  QuadrotorControlCommand struct implementation.
 *  @detail Contains information about the desired quadrotor state as control command, namely:
 *          the orientation, collective thrust, bodyrates, angular acceleration
 */
struct QuadrotorControlCommand {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


};  /* struct QuadrotorControlCommand */

} /* namespace quadrotor_common */

#endif  /* QUADROTOR_COMMON_QUADROTOR_CONTROL_COMMAND_H */
