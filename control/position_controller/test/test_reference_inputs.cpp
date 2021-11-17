/**
 *  @file   test_reference_inputs.cpp
 *  @brief  quadrotor position control's reference inputs related functionality unit tests
 *  @author thor
 *  @date   17.11.2021
 */
#include "position_controller/reference_inputs.h"

//  3rd party dependencies
#include <gtest/gtest.h>
#include <ros/ros.h>

namespace position_controller {

/**
 *  @brief  Test fixture for testing the class ReferenceInputs
 *  @detail reference: https://google.github.io/googletest/primer.html
 */
class ReferenceInputsTest : public ::testing::Test {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:

      /////////////////////////////////////////////////////
      ////////////  Constructors & Destructors  ///////////
      /////////////////////////////////////////////////////

  /**
   *  @brief  ReferenceInputsTest's default constructor, called for each test
   *          to perform setup tasks
   */
  ReferenceInputsTest() {}

  /**
   *  @brief  ReferenceInputsTest's default destructor, called for each test
   *          to perform cleanup tasks
   */
  ~ReferenceInputsTest() override {}

  ////////////////////////////////////////
  ////////////  Class Methods  ///////////
  ////////////////////////////////////////

  /**
   *  @brief  For additional setup tasks, called immediately after the constructor
   *          right before each test
   */
  void SetUp() override {}

  /**
   *  @brief  For additional cleanup tasks, called immediately after each test
   *          right before the destructor
   */
  void TearDown() override {}

}; /*  class ReferenceInputsTest  */

/**
 *  @brief  Test case
 */
TEST_F(ReferenceInputsTest, SampleTest) {

}

} /*  namespace position_controller  */

/**
 *
 */
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "test_reference_inputs");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
