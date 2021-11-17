/**
 *  @file   reference_inputs.h
 *  @brief  quadrotor position control's reference inputs related functionality declaration & definition
 *  @author thor
 *  @date   17.11.2021
 */
#ifndef POSITION_CONTROLLER_REFERENCE_INPUTS_H
#define POSITION_CONTROLLER_REFERENCE_INPUTS_H

//  3rd party dependencies
#include <Eigen/Dense>

//  quadrotor_common dependencies
#include "quadrotor_common/math.h"
#include "quadrotor_common/quadrotor_control_command.h"
#include "quadrotor_common/quadrotor_state_estimate.h"
#include "quadrotor_common/quadrotor_trajectory_point.h"

namespace position_controller {

/**
 *  @brief  ReferenceInputs class implementation
 *  @detail Compute the desirted orientation, the desired collective thrust command,
 *          the desired body rates (angular velocity) and the desires angular acceleration;
 *          required for high-level position control.
 */
class ReferenceInputs {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /////////////////////////////////////////////////////
        ////////////  Constructors & Destructors  ///////////
        /////////////////////////////////////////////////////

    /**
     *  @brief  ReferenceInputs' default constructor, called when an instance is created
     */
    ReferenceInputs(
        const quadrotor_common::QuadrotorStateEstimate& state_est,
        const quadrotor_common::QuadrotorTrajectoryPoint& state_ref);

    /**
     *  @brief  ReferenceInputs' default destructor, called when an instance is destroyed
     */
    ~ReferenceInputs();

        ////////////////////////////////////////
        ////////////  Class Methods  ///////////
        ////////////////////////////////////////

    /**
     *
     */

 private:

        ////////////////////////////////////
        ////////////  Constants  ///////////
        ////////////////////////////////////

    //  @brief  The gravity acting in -ve z_W direction i.e kGravity_ = -g.z_W
    const Eigen::Vector3d kGravity_ = Eigen::Vector3d(0.0, 0.0, -9.81);

    //  @brief  The almost zero value threshold
    static constexpr double kAlmostZeroValueThreshold_ = 0.001;

        ////////////////////////////////////////
        ////////////  Class Methods  ///////////
        ////////////////////////////////////////

    /**
     *  @brief  Compute the robust reference orientation R = [x_B, y_B, z_B]
     *  @detail for x_B refer computeRobustBodyXAxis()
     *          for y_B refer computeRobustBodyYAxis()
     */
    Eigen::Quaterniond computeReferenceOrientation() const;

    /**
     *  @brief  Compute the reference collective thrust c
     *  @param  orientation - orientation computed from computeReferenceOrientation()
     */
    double computeReferenceCollectiveThrust(
         const Eigen::Quaterniond& orientation) const;

    /**
     *  @brief  Compute the reference bodyrates omega and angular accelerations omega_dot
     *  @param  orientation - orientation computed from computeReferenceOrientation()
     *  @param  c - thrust computed from computeReferenceCollectiveThrust()
     *  @param  bodyrates     - computed bodyrates
     *  @param  bodyrates_dot - computed angular accelerations
     */
    void computeReferenceBodyratesAndDerivative(
        const Eigen::Quaterniond& orientation,
        const double c,
        Eigen::Vector3d& bodyrates,
        Eigen::Vector3d& bodyrates_dot) const;

    /**
     *  @brief  Compute robust x_B from the input constraints, where x_B = (y_C x alpha)
     *  @detail Handle singularities when y_C is aligned with alpha or alpha = 0.
     *          For an extreme case solution we set x_B = x_C.
     */
    Eigen::Vector3d computeRobustBodyXAxis() const;

    /**
     *  @brief  Compute robust y_B from the input constraints, where y_B = (beta x x_B)
     *  @detail Handle singularities when x_B is aligned with beta or beta = 0.
     *          For an extreme case solution we set y_B = y_C.
     *  @param  x_B   - robust x_B computed from computeRobustBodyXAxis()
     */
    Eigen::Vector3d computeRobustBodyYAxis(const Eigen::Vector3d& x_B) const;

        ////////////////////////////////////////
        ////////////  Class Members  ///////////
        ////////////////////////////////////////

    //  @brief  Input quadrotor's current state estimate
    quadrotor_common::QuadrotorStateEstimate state_estimate;

    //  @brief  Input quadrotor's reference state to track
    quadrotor_common::QuadrotorTrajectoryPoint reference_state;

    //  @brief  Output quadrotor's reference inputs
    quadrotor_common::QuadrotorControlCommand reference;

    //  @brief  Constraints to enforce reference heading phi
    Eigen::Vector3d x_C, y_C;

    // TODO: get the values from config
    //  @brief  Rotor drag constants
    double dx = 0, dy = 0, dz = 0;

};  /*  class ReferenceInputs  */

} /*  namespace position_controller  */

#endif  /*  POSITION_CONTROLLER_REFERENCE_INPUTS_H  */
