/**
 *  @file   math.h
 *  @brief  mathematical functionality declaration & definition
 *  @author thor
 *  @date   17.11.2021
 */
#ifndef QUADROTOR_COMMON_MATH_H
#define QUADROTOR_COMMON_MATH_H

namespace quadrotor_common {

bool isAlmostZero(const double value, const double threshold) {
  return fabs(value) < threshold;
}

} /*  namespace quadrotor_common  */

#endif  /*  QUADROTOR_COMMON_MATH_H  */
