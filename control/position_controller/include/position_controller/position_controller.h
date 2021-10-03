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

namespace position_controller {

/**
 *  @brief  PositionController class implementation.
 *  @detail The high-level position control outputs the desired orientation, the desired collective thrust command,
 *          the desired body rates, and the desired angular accelerations.
 */
class PositionController {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ///////////////////////////////////////////////////
        //////////// Constructors & Destructors ///////////
        ///////////////////////////////////////////////////

    /**
     *  @brief  PositionController's default constructor called when an instance is created.
     */
    PositionController();

    /**
     *  @brief  PositionController's default destructor called when an instace is destroyed.
     */
    ~PositionController();

 protected:
 private:
};  /* class PositionController */

} /* namespace position_controller */

#endif  /* POSITION_CONTROLLER_POSITION_CONTROLLER_H */
