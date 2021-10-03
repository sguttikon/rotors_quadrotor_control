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
#include <ros/time.h>

namespace quadrotor_common {

/**
 *  @brief  QuadrotorControlCommand struct implementation.
 *  @detail Contains information about the desired quadrotor state as control command, namely:
 *          the orientation, collective thrust, bodyrates, angular acceleration
 */
struct QuadrotorControlCommand {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      ///////////////////////////////
      //////////// Types ////////////
      ///////////////////////////////

  /**
   *  @brief  ControlMode enum implementation.
   *  @detail Different type of high level position control outputs represented with corresponding modes.
   *            + kNone                 - Unknown/Unsupported control command
   *            + kAttitude             - The desired orientation control output
   *            + kBodyrates            - The desired bodyrates control output
   *            + kAngularAcceleration  - The desired angular acceleration control output
   */
  enum class ControlMode {
    kNone,
    kAttitude,
    kBodyrates,
    kAngularAcceleration
  };  /* enum class ControlMode */

      ///////////////////////////////////////////////////
      //////////// Constructors & Destructors ///////////
      ///////////////////////////////////////////////////

  /**
   *  @brief  QuadrotorControlCommand's default constructor, called when an instance is created.
   */
  QuadrotorControlCommand();

  /**
   *  @brief  QuadrotorControlCommand's default destructor, called when an instance is destroyed.
   */
  ~QuadrotorControlCommand();

      //////////////////////////////////////
      ///////////// Data Members ///////////
      //////////////////////////////////////

  //  @brief  The timestamp of control command output
  ros::Time timestamp;

  //  @brief  The control command mode
  ControlMode control_mode;

  // TODO: w.r.t which coordinate frame the following values are calculated ?
  //  @brief  The desired quaternion orientation of quadrotor.
  Eigen::Quaterniond orientation;

  //  @brief  The desired 3d body rates of quadrotor in body coordinate frame.
  Eigen::Vector3d bodyrates;

  //  @brief  The desired angular acceleration [rad/s^2] of quadrotor.
  Eigen::Vector3d angular_acceleration;

  //  @brief  The desired mass normalized collective thrust [m/s^2] of quadrotor.
  double collective_thrust;

};  /* struct QuadrotorControlCommand */

} /* namespace quadrotor_common */

#endif  /* QUADROTOR_COMMON_QUADROTOR_CONTROL_COMMAND_H */
