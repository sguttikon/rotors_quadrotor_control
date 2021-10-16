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
          reference_state(state_ref) {

      // constraints based on reference heading, i.e.,
      // projection of x_B into x_W - y_W plane will be collinear with x_C
      const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
          Eigen::AngleAxisd(reference_state.heading, Eigen::Vector3d::UnitZ()));
      x_C = q_heading * Eigen::Vector3d::UnitX();
      y_C = q_heading * Eigen::Vector3d::UnitY();
    }

        //////////////////////////////////////
        //////////// Class Methods ///////////
        //////////////////////////////////////

    /**
     *  @brief
     *  @detail
     *  @return computed desired attitude based on quadrotor's state estimate and reference state.
     */
    virtual Eigen::Quaterniond computeDesiredAttitude() const = 0;

    /**
     *  @brief  Compute robust x_B from the input constraints, where x_B = (y_C x alpha).
     *  @detail Handle singularities when y_C is aligned with alpha or alpha = 0.
     *          For an extreme case, set x_B = x_C as default solution.
     *  @param  y_C               - projection constraint
     *  @param  alpha             - acceleration constraint
     *  @param  attitude_estimate - quadrotor state's attitude estimate
     *  @param  x_C               - projection constraint (default solution)
     *  @return computed x_B that statisfies all required constraints.
     */
    Eigen::Vector3d computeRobustBodyXAxis(
        const Eigen::Vector3d& y_C,
        const Eigen::Vector3d& alpha,
        const Eigen::Quaterniond& attitude_estimate,
        const Eigen::Vector3d& x_C) const;

    /**
     *  @brief  Compute robust y_B from the input constraints, where y_B = (beta x x_B).
     *  @detail Handle singularities when x_B is aligned with beta or beta = 0.
     *          For an extreme case, set y_B = y_C as default solution.
     *  @param  x_B               - robust computed body x axis from computeRobustBodyXAxis()
     *  @param  beta              - acceleration constraint
     *  @param  attitude_estimate - quadrotor state's attitude estimate
     *  @param  y_C               - projection constraint (default solution)
     *  @return computed y_B that statisfies all required constraints.
     */
    Eigen::Vector3d computeRobustBodyYAxis(
        const Eigen::Vector3d& x_B,
        const Eigen::Vector3d& beta,
        const Eigen::Quaterniond& attitude_estimate,
        const Eigen::Vector3d& y_C) const;

 private:

        //////////////////////////////////
        //////////// Constants ///////////
        //////////////////////////////////

    //  @brief  The gravity in -ve z_W direction
    const Eigen::Vector3d kGravity_ = Eigen::Vector3d(0.0, 0.0, -9.81);

    //  @brief
    static constexpr double kAlmostZeroValueThreshold_ = 0.001;

        //////////////////////////////////////
        //////////// Class Methods ///////////
        //////////////////////////////////////

    /**
     *  @brief  Check if the input value is below the almost zero value threshold or not.
     *  @param  value   - input value to check
     *  @return boolean value where
     *            + true  - Indicates the input value is almost zero
     *            + false - Otherwise
     */
    bool isAlmostZero(const double value) const;

        //////////////////////////////////////
        //////////// Class Members ///////////
        //////////////////////////////////////

    //  @brief
    quadrotor_common::QuadrotorTrajectoryPoint reference_state;

    //  @brief
    quadrotor_common::QuadrotorStateEstimate state_estimate;

    //  @brief
    Eigen::Vector3d x_C, y_C;

};  /* class ReferenceInputs */

} /* namespace position_controller */

#endif  /* REFERENCE_INPUTS_REFERENCE_INPUTS_H */
