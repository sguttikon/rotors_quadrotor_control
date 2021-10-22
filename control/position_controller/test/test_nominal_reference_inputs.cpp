/**
 *  @file   test_position_controller.cpp
 *  @brief  quadrotor position control's nominal dynamics reference inputs related functionality unit tests
 *  @author neo
 *  @data   16.10.2021
 */
#include "reference_inputs/nominal_reference_inputs.h"

// 3rd party dependencies
#include <gtest/gtest.h>
#include <ros/ros.h>

namespace position_controller {

/**
 *  @brief  Test fixture for testing the class NominalReferenceInputs.
 *  @detail
 */
class NominalReferenceInputsTest : public ::testing::Test {
 protected:
        ///////////////////////////////////////////////////
        //////////// Constructors & Destructors ///////////
        ///////////////////////////////////////////////////

    /**
     *  @brief NominalReferenceInputsTest's default constructor, called for each test to do set-up work.
     */
    NominalReferenceInputsTest() {}

    /**
     *  @brief NominalReferenceInputsTest's default destructor, called for each test to do clean-up work.
     */
    ~NominalReferenceInputsTest() override {}

        //////////////////////////////////////
        //////////// Class Methods ///////////
        //////////////////////////////////////

    /**
     *  @brief For additional set-up work, called immediately after the constructor right before each test.
     */
    void SetUp() override {}

    /**
     *  @brief For additional clean-up work, called immediately after each test right before the destructor.
     */
    void TearDown() override {}

        //////////////////////////////////////
        //////////// Class Members ///////////
        //////////////////////////////////////

    std::unique_ptr<quadrotor_common::QuadrotorStateEstimate> state_est_ptr = std::make_unique<quadrotor_common::QuadrotorStateEstimate>();
    std::unique_ptr<quadrotor_common::QuadrotorTrajectoryPoint> state_ref_ptr = std::make_unique<quadrotor_common::QuadrotorTrajectoryPoint>();
    std::unique_ptr<NominalReferenceInputs> reference_inputs_ptr;

};  /* class NominalReferenceInputsTest */

/**
 *  @brief  Test case to check if instance of NominalReferenceInputs can be created or not.
 */
TEST_F(NominalReferenceInputsTest, InitializationTest) {
  EXPECT_EQ(reference_inputs_ptr, nullptr);

  reference_inputs_ptr.reset(new NominalReferenceInputs(*state_est_ptr, *state_ref_ptr));
  EXPECT_NE(reference_inputs_ptr, nullptr);

  reference_inputs_ptr.release();
  EXPECT_EQ(reference_inputs_ptr, nullptr);
}

/**
 *  @brief  Test case to check if robust body xaxis is calculated correctly.
 */
TEST_F(NominalReferenceInputsTest, ComputeRobustBodyXAxisTest) {
  Eigen::Vector3d x_B, x_B_gt, alpha;

  // Case: x_B = normalized(x_B_est - (x_B_est^T y_C)y_C)
  reference_inputs_ptr.reset(new NominalReferenceInputs(*state_est_ptr, *state_ref_ptr));
  x_B = reference_inputs_ptr->computeRobustBodyXAxis(
      reference_inputs_ptr->getY_C(), alpha, state_est_ptr->orientation, reference_inputs_ptr->getX_C());
  x_B_gt = Eigen::Vector3d(1., 0., 0.);
  EXPECT_TRUE(x_B_gt.isApprox(x_B));

  // Case: x_B = normalized(y_C x alpha)
  reference_inputs_ptr.reset(new NominalReferenceInputs(*state_est_ptr, *state_ref_ptr));
  alpha = Eigen::Vector3d::UnitX();
  x_B = reference_inputs_ptr->computeRobustBodyXAxis(
      reference_inputs_ptr->getY_C(), alpha, state_est_ptr->orientation, reference_inputs_ptr->getX_C());
  x_B_gt = Eigen::Vector3d(0., 0., -1.);
  EXPECT_TRUE(x_B_gt.isApprox(x_B));

  // Case: x_B = x_C
  // TODO
}

/**
 *  @brief
 */
TEST_F(NominalReferenceInputsTest, ComputeRobustBodyYAxisTest) {
  Eigen::Vector3d x_B, y_B, y_B_gt, beta;

  // Case: y_B = y_C
  reference_inputs_ptr.reset(new NominalReferenceInputs(*state_est_ptr, *state_ref_ptr));
  y_B = reference_inputs_ptr->computeRobustBodyYAxis(
      x_B, beta, state_est_ptr->orientation, Eigen::Vector3d::UnitX());
  y_B_gt = Eigen::Vector3d(1., 0., 0.);
  EXPECT_TRUE(y_B_gt.isApprox(y_B));

  //  Case: y_B = normalized(z_B_est x x_B)
  reference_inputs_ptr.reset(new NominalReferenceInputs(*state_est_ptr, *state_ref_ptr));
  x_B = Eigen::Vector3d::UnitX();
  y_B = reference_inputs_ptr->computeRobustBodyYAxis(
      x_B, beta, state_est_ptr->orientation, reference_inputs_ptr->getY_C());
  y_B_gt = Eigen::Vector3d(0., 1., 0.);
  EXPECT_TRUE(y_B_gt.isApprox(y_B));

  //  Case: y_B = normalized(beta x x_B)
  reference_inputs_ptr.reset(new NominalReferenceInputs(*state_est_ptr, *state_ref_ptr));
  x_B = Eigen::Vector3d::UnitX();
  beta = Eigen::Vector3d::UnitY();
  y_B = reference_inputs_ptr->computeRobustBodyYAxis(
      x_B, beta, state_est_ptr->orientation, reference_inputs_ptr->getY_C());
  y_B_gt = Eigen::Vector3d(0., 0., -1.);
  EXPECT_TRUE(y_B_gt.isApprox(y_B));
}

} /* namespace position_controller */

/**
 *  @brief
 */
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "test_nominal_reference_inputs");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
