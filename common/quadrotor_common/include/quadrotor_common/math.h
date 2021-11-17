/**
 *  @file   math.h
 *  @brief  mathematical functionality declaration & definition
 *  @author thor
 *  @date   17.11.2021
 */
#ifndef QUADROTOR_COMMON_MATH_H
#define QUADROTOR_COMMON_MATH_H

//  3rd party dependencies
#include <Eigen/Dense>

namespace quadrotor_common {

bool isAlmostZero(const double value, const double threshold) {
  return fabs(value) < threshold;
}

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d v_hat;
  v_hat <<    0.0, -v.z(),  v.y(),
            v.z(),    0.0, -v.x(),
           -v.y(),  v.x(),    0.0;
  return v_hat;
}

} /*  namespace quadrotor_common  */

#endif  /*  QUADROTOR_COMMON_MATH_H  */
