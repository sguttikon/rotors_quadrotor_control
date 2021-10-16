/**
 *  @file   nominal_reference_inputs.h
 *  @brief  quadrotor position control's nominal dynamics reference inputs related functionality declaration & definition
 *  @author neo
 *  @data   16.10.2021
 */
#ifndef REFERENCE_INPUTS_NOMINAL_REFERENCE_INPUTS_H
#define REFERENCE_INPUTS_NOMINAL_REFERENCE_INPUTS_H

// 3rd party dependencies
#include <Eigen/Dense>

// position_controller dependencies
#include "reference_inputs/reference_inputs.h"

namespace position_controller {

/**
 *  @brief  NominalReferenceInputs class implementation.
 *  @detail
 */
class NominalReferenceInputs : public ReferenceInputs {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ///////////////////////////////////////////////////
        //////////// Constructors & Destructors ///////////
        ///////////////////////////////////////////////////

    /**
     *  @brief  NominalReferenceInputs's default constructor, called when an instance is created.
     */
    NominalReferenceInputs(
        const quadrotor_common::QuadrotorStateEstimate& state_est,
        const quadrotor_common::QuadrotorTrajectoryPoint& state_ref)
        : ReferenceInputs(state_est, state_ref) {}

    /**
     *  @brief  NominalReferenceInputs's default destructor, called when an instance is destroyed.
     */
    ~NominalReferenceInputs() {}

    /**
     *  @brief
     *  @detail
     *  @param  reference_state   -
     *  @param  attitude_estimate -
     *  @return desired_attitude  -
     */
    Eigen::Quaterniond computeDesiredAttitude() const override {}

};  /* class NominalReferenceInputs */

} /* namespace position_controller */

#endif  /* REFERENCE_INPUTS_NOMINAL_REFERENCE_INPUTS_H */
