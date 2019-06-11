/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Bryce Willey.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of MoveIt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Bryce Willey */

// The Testing Framework and Utils
#include <moveit/robot_model/robot_model.h>
#include <gtest/gtest.h>
#include <moveit/utils/robot_model_test_utils.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// BEGIN_TUTORIAL
// Google Test has two kinds of testing functions. The first, *TEST*, simply runs the code inside.
// It takes two names: the name of the test case, also called a test suite, which is a group of several tests that are
// all related to the same component, and the name of the test.
// To actually check that your code does what is expected, you can add assertions.
// ASSERT will stop the test if it fails, and EXPECT will continue running, although the test will still fail.

TEST(MyFirstTestCase, TestName)
{
  int x = 0;
  x = 5 * 10 + 4;

  ASSERT_NE(x, 0); /* NE is 'not equal' */
  EXPECT_EQ(x, 54);
}

// A Test Fixture loads the same data repeatedly for multiple tests, so common setup code doesn't have to be duplicated.
// To make a test fixture, first make a class that derives from ::testing::Test.
// You can use either the constructor or SetUp to load the information.
// Let's use MoveIt's testing utilities to load a robot model.

class MyTestFixture : public ::testing::Test
{
  /* Everything in the class can be protected:. */
protected:
  void SetUp() override
  {
    robot_model_ = moveit::core::loadTestingRobotModel("panda_description");
  }

  /* If you need to cleanup the resources any tests are using, you can do it in TearDown(). */
  void TearDown() override
  {
  }

  moveit::core::RobotModelConstPtr robot_model_;
};

// To make a test that uses the data loaded by this fixture, use TEST_F.
TEST_F(MyTestFixture, InitOK)
{
  ASSERT_EQ(robot_model_->getURDF()->getName(), "panda");
  ASSERT_EQ(robot_model_->getSRDF()->getName(), "panda");
}

// MoveIt also provides a RobotModelBuilder class that helps you build simple robots with the link and joint structure
// that you need for a specific test.
// Let's make a robot that has one branch, making a Y shape.

TEST(MyFirstTestCase, SimpleYRobot)
{
  moveit::core::RobotModelBuilder builder("one_robot", "base_link");
  builder.addChain("base_link->a", "continuous");
  builder.addChain("a->b->c", "fixed");
  builder.addChain("a->d", "fixed");
  builder.addChain("d->e", "continuous");
  builder.addVirtualJoint("odom", "base_link", "planar", "base_joint");
  builder.addGroup({}, { "base_joint" }, "base_joint");
  ASSERT_TRUE(builder.isValid());
  moveit::core::RobotModelConstPtr robot_model = builder.build();

  /* Let's check that the link c is rigidly connected to link d, as it should be. */
  const moveit::core::LinkTransformMap transforms = robot_model->getLinkModel("c")->getAssociatedFixedTransforms();
  auto maybe_link_d = map.find(robot_model->getLinkModel("d"));
  ASSERT_NE(maybe_link_d, map.end());
}

// END_TUTORIAL
