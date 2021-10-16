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

};  /* class NominalReferenceInputsTest */

/**
 *  @brief
 */
TEST_F(NominalReferenceInputsTest, testCase1) {
  EXPECT_EQ(0, 0);
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
