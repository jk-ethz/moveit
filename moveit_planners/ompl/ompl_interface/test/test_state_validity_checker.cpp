
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, KU Leuven
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Jeroen De Maeyer */

/** \brief Use this flag to turn on extra output on std::cout for debugging. **/
constexpr bool VERBOSE{ true };

#include <moveit/ompl_interface/detail/state_validity_checker.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h>

// #include <memory>
// #include <string>
// #include <iostream>
#include <limits>
#include <ostream>

#include <gtest/gtest.h>
// #include <Eigen/Dense>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/Constraints.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
// #include <moveit/planning_interface/planning_request.h>

#include <ompl/geometric/SimpleSetup.h>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>

// #include <ompl/util/Exception.h>
// #include <ompl/base/spaces/RealVectorStateSpace.h>
// #include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

/** \brief Pretty print std:vectors **/
std::ostream& operator<<(std::ostream& os, const std::vector<double>& v)
{
  os << "( ";
  for (auto value : v)
    os << value << ", ";
  os << " )";
  return os;
}

/** \brief Helper function to create a specific position constraint. **/
moveit_msgs::PositionConstraint createPositionConstraint(const std::string& base_link, const std::string& ee_link_name)
{
  shape_msgs::SolidPrimitive box_constraint;
  box_constraint.type = shape_msgs::SolidPrimitive::BOX;
  // box_constraint.dimensions = { 0.05, 0.4, 0.05 };

  box_constraint.dimensions.resize(3);
  box_constraint.dimensions[box_constraint.BOX_X] = 0.05;
  box_constraint.dimensions[box_constraint.BOX_Y] = 0.04;
  box_constraint.dimensions[box_constraint.BOX_Z] = 0.05;

  geometry_msgs::Pose box_pose;
  box_pose.position.x = 0.3;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.5;
  box_pose.orientation.w = 1.0;

  moveit_msgs::PositionConstraint position_constraint;
  position_constraint.header.frame_id = base_link;
  position_constraint.link_name = ee_link_name;
  position_constraint.constraint_region.primitives.push_back(box_constraint);
  position_constraint.constraint_region.primitive_poses.push_back(box_pose);

  return position_constraint;
}

/** \brief Robot indepentent test class implementing all tests
 *
 * All tests are implemented in a generic test fixture, so it is
 * easy to run them on different robots.
 *
 * based on
 * https://stackoverflow.com/questions/38207346/specify-constructor-arguments-for-a-google-test-fixture/38218657
 * (answer by PiotrNycz)
 *
 * It is implemented this way to avoid the ros specific test framework
 * outside moveit_ros.
 *
 * (This is an (uglier) alternative to using the rostest framework
 * and reading the robot settings from the parameter server.
 * Then we have several rostest launch files that load the parameters
 * for a specific robot and run the same compiled tests for all robots.)
 * */
class LoadTestRobotBaseClass : public testing::Test
{
protected:
  LoadTestRobotBaseClass(const std::string& robot_name, const std::string& group_name)
    : group_name_(group_name), robot_name_(robot_name)
  {
  }

  void SetUp() override
  {
    // load robot
    robot_model_ = moveit::core::loadTestingRobotModel(robot_name_);
    robot_state_ = std::make_shared<robot_state::RobotState>(robot_model_);
    robot_state_->setToDefaultValues();
    start_state_ = std::make_shared<robot_state::RobotState>(robot_model_);
    start_state_->setToDefaultValues();
    joint_model_group_ = robot_state_->getJointModelGroup(group_name_);

    // extract useful parameters for tests
    num_dofs_ = joint_model_group_->getVariableCount();
    ee_link_name_ = joint_model_group_->getLinkModelNames().back();
    base_link_name_ = robot_model_->getRootLinkName();

    // setup all the input we need to create a StateValidityChecker
    setupMoveItStateSpace();
    setupPlanningContext();
  };

  void TearDown() override
  {
  }

  void setupMoveItStateSpace()
  {
    ompl_interface::ModelBasedStateSpaceSpecification space_spec(robot_model_, group_name_);
    state_space_ = std::make_shared<ompl_interface::JointModelStateSpace>(space_spec);
    state_space_->computeLocations();  // this gets called in the state space factory normally
  }

  void setupPlanningContext()
  {
    ASSERT_NE(state_space_, nullptr) << "Initialize state space before creating the planning context.";
    planning_context_spec_.state_space_ = state_space_;
    planning_context_spec_.ompl_simple_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(state_space_);
    planning_context_ =
        std::make_shared<ompl_interface::ModelBasedPlanningContext>(group_name_, planning_context_spec_);

    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    planning_context_->setPlanningScene(planning_scene_);
    planning_context_->setCompleteInitialState(*start_state_);

    if (VERBOSE)
      ROS_INFO("Planning context with name %s is ready (but not configured).", planning_context_->getName().c_str());
  }

protected:
  const std::string group_name_;
  const std::string robot_name_;

  moveit::core::RobotModelPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  robot_state::RobotStatePtr start_state_;
  const robot_state::JointModelGroup* joint_model_group_;

  std::size_t num_dofs_;
  std::string base_link_name_;
  std::string ee_link_name_;

  ompl_interface::ModelBasedStateSpacePtr state_space_;
  ompl_interface::ModelBasedPlanningContextSpecification planning_context_spec_;
  ompl_interface::ModelBasedPlanningContextPtr planning_context_;
  planning_scene::PlanningScenePtr planning_scene_;
};

// /***************************************************************************
//  * Run all tests on the Panda robot
//  * ************************************************************************/
class PandaValidityCheckerTests : public LoadTestRobotBaseClass
{
protected:
  PandaValidityCheckerTests() : LoadTestRobotBaseClass("panda", "panda_arm")
  {
  }
};

TEST_F(PandaValidityCheckerTests, createStateValidityChecker)
{
  ompl::base::StateValidityCheckerPtr checker =
      std::make_shared<ompl_interface::StateValidityChecker>(planning_context_.get());
}

TEST_F(PandaValidityCheckerTests, testJointLimits)
{
  // std::vector<double> joint_values{ -1.87784, 1.38894, -0.829546, -1.71882, -0.579079, 2.9616, -0.887174 };
  // robot_state_->setJointGroupPositions(joint_model_group_, joint_values);

  auto checker = std::make_shared<ompl_interface::StateValidityChecker>(planning_context_.get());
  checker->setVerbose(VERBOSE);

  // use a scoped OMPL state so we don't have to call allocState and freeState
  // (as recommended in the OMPL documantion)
  ompl::base::ScopedState<> ompl_state(state_space_);

  robot_state_->setToDefaultValues();
  state_space_->copyToOMPLState(ompl_state.get(), *robot_state_);

  if (VERBOSE)
    ROS_INFO_STREAM(ompl_state.reals());

  // assume the default position in not in self-collision
  // and there are no collision objects of path constraints so this state should be valid
  bool result = checker->isValid(ompl_state.get());
  EXPECT_TRUE(result);

  // move first joint obviously outside any joint limits
  ompl_state->as<ompl_interface::JointModelStateSpace::StateType>()->values[0] = std::numeric_limits<double>::max();
  ompl_state->as<ompl_interface::JointModelStateSpace::StateType>()->clearKnownInformation();

  if (VERBOSE)
    ROS_INFO_STREAM(ompl_state.reals());

  bool result_2 = checker->isValid(ompl_state.get());
  EXPECT_FALSE(result_2);
}

TEST_F(PandaValidityCheckerTests, testPathConstraints)
{
  ASSERT_NE(planning_context_, nullptr) << "Initialize planning context before adding path constraints.";

  moveit_msgs::Constraints path_constraints;
  path_constraints.name = "test_position_constraints";
  path_constraints.position_constraints.push_back(createPositionConstraint(base_link_name_, ee_link_name_));

  // the code below is for debugging, the code I actually want to run is in comments at the bottom
  kinematic_constraints::KinematicConstraintSet ks(robot_model_);
  const moveit::core::Transforms& tfs = planning_scene_->getTransforms();

  // this is the call where it goes wrong
  ks.add(path_constraints, tfs);

  // moveit_msgs::MoveItErrorCodes error_code_not_used;
  // bool succeeded = planning_context_->setPathConstraints(path_constraints, &error_code_not_used);
  // ASSERT_TRUE(succeeded) << "Failed to set path constraints on the planning context";
  // auto checker = std::make_shared<ompl_interface::StateValidityChecker>(planning_context_.get());
  // checker->setVerbose(VERBOSE);
}

/***************************************************************************
 * MAIN
 * ************************************************************************/
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}