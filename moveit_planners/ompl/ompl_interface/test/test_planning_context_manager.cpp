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

#include "utilities.h"

// #include <limits>
// #include <ostream>

#include <gtest/gtest.h>

#include <moveit/ompl_interface/planning_context_manager.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>

#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h>

/** \brief Use this flag to turn on extra output on std::cout for debugging. **/
// constexpr bool VERBOSE{ true };

/** \brief Generic implementation of the tests that can be executed on different robots. **/
class TestPlanningContext : public ompl_interface_testing::LoadTestRobot, public testing::Test
{
public:
  TestPlanningContext(const std::string& robot_name, const std::string& group_name)
    : LoadTestRobot(robot_name, group_name)
  {
  }

  // /***************************************************************************
  //  * START Test implementations
  //  * ************************************************************************/

  void dummy()
  {
    EXPECT_TRUE(true);

    ompl_interface::PlanningContextManager pcm(robot_model_, constraint_sampler_manager_);

    planning_interface::PlannerConfigurationSettings pconfig_settings;
    pconfig_settings.group = group_name_;
    pconfig_settings.name = group_name_;
    pconfig_settings.config = { { "enforce_joint_model_state_space", "0" }, { "use_ompl_constrained_planning", "0" } };
    planning_interface::PlannerConfigurationMap pconfig_map{ { pconfig_settings.name, pconfig_settings } };

    pcm.setPlannerConfigurations(pconfig_map);

    auto request = createRequest({ 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0.1 });

    moveit_msgs::MoveItErrorCodes error_code;

    auto pc = pcm.getPlanningContext(planning_scene_, request, error_code, node_handle_, false);

    auto ss = dynamic_cast<ompl_interface::JointModelStateSpace*>(pc->getOMPLStateSpace().get());
    EXPECT_NE(ss, nullptr);
  }

  // /***************************************************************************
  //  * END Test implementation
  //  * ************************************************************************/

protected:
  void SetUp() override
  {
    constraint_sampler_manager_ = std::make_shared<constraint_samplers::ConstraintSamplerManager>();
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
  }

  void TearDown() override
  {
  }

  /** Create a planning request to plan from a given start state to a joint space goal. **/
  planning_interface::MotionPlanRequest createRequest(const std::vector<double>& start, const std::vector<double>& goal)
  {
    planning_interface::MotionPlanRequest request;

    request.group_name = group_name_;
    request.allowed_planning_time = 5.0;

    // fill out start state in request
    robot_state::RobotState start_state(robot_model_);
    start_state.setToDefaultValues();
    start_state.setJointGroupPositions(joint_model_group_, start);
    moveit::core::robotStateToRobotStateMsg(start_state, request.start_state);

    // fill out goal state in request
    robot_state::RobotState goal_state(robot_model_);
    goal_state.setToDefaultValues();
    goal_state.setJointGroupPositions(joint_model_group_, goal);
    moveit_msgs::Constraints joint_goal =
        kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group_, 0.001);
    request.goal_constraints.push_back(joint_goal);

    return request;
  }

  ompl_interface::ModelBasedStateSpacePtr state_space_;
  ompl_interface::ModelBasedPlanningContextSpecification planning_context_spec_;
  ompl_interface::ModelBasedPlanningContextPtr planning_context_;
  planning_scene::PlanningScenePtr planning_scene_;

  constraint_samplers::ConstraintSamplerManagerPtr constraint_sampler_manager_;

  ros::NodeHandle node_handle_;
};

/***************************************************************************
 * Run all tests on the Fanuc robot
 * ************************************************************************/
class FanucTestPlanningContext : public TestPlanningContext
{
protected:
  FanucTestPlanningContext() : TestPlanningContext("fanuc", "manipulator")
  {
  }
};

TEST_F(FanucTestPlanningContext, dummyTest)
{
  dummy();
}

/***************************************************************************
 * MAIN
 * ************************************************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "planning_context_manager_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}