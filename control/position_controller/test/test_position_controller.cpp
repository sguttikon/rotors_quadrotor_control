/**
 *  @file   test_position_controller.cpp
 *  @brief  quadrotor's position control related functionality unit tests
 *  @author neo
 *  @data   16.10.2021
 */
#include "position_controller/position_controller.h"

// 3rd party dependencies
#include <gtest/gtest.h>
#include <ros/ros.h>

namespace position_controller {

/**
 *  @brief
 *  @detail
 */
class PositionControllerTest : public ::testing::Test {
 protected:
        ///////////////////////////////////////////////////
        //////////// Constructors & Destructors ///////////
        ///////////////////////////////////////////////////

    /**
     *  @brief
     */
    PositionControllerTest() {}
    /**
     *  @brief
     */
    ~PositionControllerTest() override {}

        //////////////////////////////////////
        //////////// Class Methods ///////////
        //////////////////////////////////////

    /**
     *  @brief
     */
    void SetUp() override {}

    /**
     *  @brief
     */
    void TearDown() override {}

        //////////////////////////////////////
        //////////// Class Members ///////////
        //////////////////////////////////////

};  /* class PositionControllerTest */

/**
 *  @brief
 */
TEST_F(PositionControllerTest, testCase1) {
  EXPECT_EQ(0, 0);
}

} /* namespace position_controller */

/**
 *  @brief
 */
int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "test_position_controller");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
