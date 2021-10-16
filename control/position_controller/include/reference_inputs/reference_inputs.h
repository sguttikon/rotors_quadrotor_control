/**
 *  @file   reference_inputs.h
 *  @brief  quadrotor position control's reference inputs related functionality declaration & definition
 *  @author neo
 *  @data   16.10.2021
 */
#ifndef REFERENCE_INPUTS_REFERENCE_INPUTS_H
#define REFERENCE_INPUTS_REFERENCE_INPUTS_H

// 3rd party dependencies
#include <Eigen/Dense>

// quadrotor_common dependencies
#include "quadrotor_common/quadrotor_state_estimate.h"
#include "quadrotor_common/quadrotor_trajectory_point.h"

namespace position_controller {

/**
 *  @brief  ReferenceInputs class implementation.
 *  @detail Compute the desired orientation, the desired collective thrust command,
 *          the desired body rates, and the desired angular acceleration,
 *          required for high-level position controller.
 *          The key functionalities are implemented maninly in NominalReferenceInputs and
 *          AeroCompensatedReferenceInputs concrete classes.
 */
class ReferenceInputs {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ///////////////////////////////////////////////////
        //////////// Constructors & Destructors ///////////
        ///////////////////////////////////////////////////

    /**
     *  @brief  ReferenceInputs's default constructor, called when an concrete class's instance is created.
     */
    ReferenceInputs(
        const quadrotor_common::QuadrotorStateEstimate& state_est,
        const quadrotor_common::QuadrotorTrajectoryPoint& state_ref)
        : state_estimate(state_est),
          reference_state(state_ref) {}

        //////////////////////////////////////
        //////////// Class Methods ///////////
        //////////////////////////////////////

    /**
     *  @brief
     *  @detail
     *  @param  reference_state   -
     *  @param  attitude_estimate -
     *  @return desired_attitude  -
     */
    virtual Eigen::Quaterniond computeDesiredAttitude() const = 0;

private:

  //  @brief
  quadrotor_common::QuadrotorTrajectoryPoint reference_state;

  //  @brief
  quadrotor_common::QuadrotorStateEstimate state_estimate;

};  /* class ReferenceInputs */

} /* namespace position_controller */

#endif  /* REFERENCE_INPUTS_REFERENCE_INPUTS_H */
