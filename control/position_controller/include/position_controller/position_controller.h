/**
 *  @file   position_controller.h
 *  @brief  quadrotor's position control related functionality declaration & definition
 *  @author neo
 *  @data   03.10.2021
 */
#ifndef POSITION_CONTROLLER_POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_POSITION_CONTROLLER_H

// 3rd party dependencies
#include <Eigen/Dense>

// quadrotor_common dependencies
#include "quadrotor_common/quadrotor_control_command.h"
#include "quadrotor_common/quadrotor_state_estimate.h"
#include "quadrotor_common/quadrotor_trajectory_point.h"

namespace position_controller {

/**
 *  @brief  PositionController class implementation.
 *  @detail The high-level position control outputs the desired orientation, the desired collective thrust command,
 *          the desired body rates, and the desired angular acceleration.
 *          Let {x_W, y_W, z_W} represents world frame W,
 *              {x_B, y_B, z_B} represents body frame B expressed in world coordinate frame and
 *              {x_C, y_C, z_C} represents intermediate frame C obtained by rotation W by angle = quadrotor orientation.
 *          for further information:
 *            + Theory behind algorithms
 *              https://github.com/uzh-rpg/rpg_quadrotor_control/blob/master/documents/theory_and_math/theory_and_math.pdf
 *            + Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag for Accurate Tracking of
 *              High-Speed Trajectories https://arxiv.org/pdf/1712.02402.pdf
 *            + Thrust Mixing, Saturation, and Body-Rate Control for Accurate Aggressive Quadrotor Flight
 *              https://www.ifi.uzh.ch/dam/jcr:5f3668fe-1d4e-4c2b-a190-8f5608f40cf3/RAL16_Faessler.pdf
 */
class PositionController {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ///////////////////////////////////////////////////
        //////////// Constructors & Destructors ///////////
        ///////////////////////////////////////////////////

    /**
     *  @brief  PositionController's default constructor, called when an instance is created.
     */
    PositionController();

    /**
     *  @brief  PositionController's default destructor, called when an instance is destroyed.
     */
    ~PositionController();

        //////////////////////////////////////
        //////////// Class Methods ///////////
        //////////////////////////////////////

    /**
     *  @brief  Compute the high level position control outputs to track input trajectory point.
     *  @detail To achieve accurate tracking of a reference trajectory point, the position controller takes the reference
     *          inputs as feed-forward terms. Using the differential flatness property of quadrotor dynamics
     *          subject to rotor drag we can compute required reference (desired) quantities:
     *            orientation, thrust, bodyrates and angular acceleration.
     *  @param  state_estimate    -
     *  @param  reference_state   -
     *  @return control_command   -
     */
    quadrotor_common::QuadrotorControlCommand run(
        const quadrotor_common::QuadrotorStateEstimate& state_estimate,
        const quadrotor_common::QuadrotorTrajectoryPoint& reference_state);

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
     *  @brief
     *  @detail
     *  @param  reference_state   -
     *  @param  attitude_estimate -
     *  @return  control_command  -
     */
    quadrotor_common::QuadrotorControlCommand computeNominalReferenceInputs(
        const quadrotor_common::QuadrotorTrajectoryPoint& reference_state,
        const Eigen::Quaterniond& attitude_estimate) const;

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

    /**
     *  @brief  Check if the input value is below the almost zero value threshold or not.
     *  @param  value   - input value to check
     *  @return boolean value where
     *            + true  - Indicates the input value is almost zero
     *            + false - Otherwise
     */
    bool isAlmostZero(const double value) const;

};  /* class PositionController */

} /* namespace position_controller */

#endif  /* POSITION_CONTROLLER_POSITION_CONTROLLER_H */
