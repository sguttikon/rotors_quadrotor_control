/**
 *  @file   state_estimate.h
 *  @brief  quadrotor's state estimation related functionality declaration & definition
 *  @author neo
 *  @data   03.10.2021
 */
#ifndef QUADROTOR_COMMON_QUATERNION_STATE_ESTIMATE_H
#define QUADROTOR_COMMON_QUATERNION_STATE_ESTIMATE_H

// 3rd party dependencies
#include <Eigen/Dense>
#include <ros/time.h>

namespace quadrotor_common {

/**
 *  @brief  QuadrotorStateEstimate class implementation.
 *  @detail Contains information about quadrotor's state w.r.t coordinate frame, namely:
 *          the 3d position, 3d velocity, quaternion orientation, 3d bodyrates
 */
struct QuadrotorStateEstimate {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      ///////////////////////////////
      //////////// Types ////////////
      ///////////////////////////////

  /**
   *  @brief  CoordinateFrame enum implementation.
   *  @detail Different types of coordinate frames w.r.t which the state of quadrotor is estimated.
   *            + kInvalid    - Unknown/Unsupported coordinate frame
   *            + kWorld      - Simulation world
   */
  enum class CoordinateFrame {
    kInvalid,
    kWorld
  };  /* enum class CoordinateFrame */

      ///////////////////////////////////////////////////
      //////////// Constructors & Destructors ///////////
      ///////////////////////////////////////////////////

  /**
   *  @brief QuadrotorStateEstimate's default constructor called when an instance is created.
   */
  QuadrotorStateEstimate();

  /**
   *  @brief QuadrotorStateEstimate's default destructor called when an instance is destroyed.
   */
  ~QuadrotorStateEstimate();

      //////////////////////////////////////
      //////////// Data Members ///////////
      //////////////////////////////////////

  //  @brief  The timestamp of quadrotor's state estimate
  ros::Time timestamp;

  //  @brief  The coordinate frame w.r.t the quadrotor's state is estimated.
  CoordinateFrame coordinate_frame;

  //  @brief  The 3d position of quadrotor in coordinate frame.
  Eigen::Vector3d position;

  //  @brief The quaternion orientation of quadrotor in coordinate frame.
  Eigen::Quaterniond orientation;

  //  @brief  The 3d position's derivative of quadrotor in coordinate frame.
  Eigen::Vector3d velocity;

  //  @brief  The 3d orientation's derivative of quadrotor in body coordinate frame.
  Eigen::Vector3d bodyrates;

};  /* class QuadrotorStateEstimate */

} /* namespace quadrotor_common */

#endif  /* QUADROTOR_COMMON_QUATERNION_STATE_ESTIMATE_H */
