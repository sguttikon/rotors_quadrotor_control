/**
 *  @file   quadrotor_trajectory_point.h
 *  @brief  quadrotor's state along trajectory related functionality declaration & definition
 *  @author neo
 *  @data   03.10.2021
 */
#ifndef QUADROTOR_COMMON_QUADROTOR_TRAJECTORY_POINT_H
#define QUADROTOR_COMMON_QUADROTOR_TRAJECTORY_POINT_H

// 3rd party dependencies
#include <Eigen/Dense>

namespace quadrotor_common {

/**
 *  @brief  QuadrotorTrajectoryPoint struct implementation.
 *  @detail Contains information about quadrotor'state at given instance of a trajectory., namely:
 *          the 3d position, quaternion orientation, heading and respective time derivatives
 */
struct QuadrotorTrajectoryPoint {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      ///////////////////////////////////////////////////
      //////////// Constructors & Destructors ///////////
      ///////////////////////////////////////////////////

  /**
   *  @brief  QuadrotorTrajectoryPoint's default constructor, called when an instance is created.
   */
  QuadrotorTrajectoryPoint();

  /**
   *  @brief QuadrotorTrajectoryPoint's default destructor, called when an instance is destroyed-
   */
  ~QuadrotorTrajectoryPoint();

      //////////////////////////////////////
      //////////// Data Members ///////////
      //////////////////////////////////////

  //  @brief  The 3d position [m] of quadrotor at given instance of a trajectory.
  Eigen::Vector3d position;

  //  @brief  The quaternion orientation [-] of quadrotor at given instance of a trajectory.
  Eigen::Quaterniond orientation;

  // TODO: w.r.t which coordinate frame the following values are calculated ?
  //  @brief  The quadrotor's heading angle [rad] at given instance of a trajectory.
  double heading;

  //  @brief  The 3d linear time derivatives of quadrotor.
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  Eigen::Vector3d jerk;
  Eigen::Vector3d snap;

  //  @brief  The 3d angular time derivatives of quadrotor.
  Eigen::Vector3d bodyrates;
  Eigen::Vector3d angular_acceleration;
  Eigen::Vector3d angular_jerk;
  Eigen::Vector3d angular_snap;

  //  @brief  The angle time derivatives of quadrotor's heading
  double heading_rate;
  double heading_acceleration;

};  /* struct QuadrotorTrajectoryPoint */

} /* namespace quadrotor_common */

#endif  /* QUADROTOR_COMMON_QUADROTOR_TRAJECTORY_POINT_H */
