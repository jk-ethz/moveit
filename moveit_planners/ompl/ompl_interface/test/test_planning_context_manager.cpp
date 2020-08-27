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
 *   * Neither the name of KU Leuven nor the names of its
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

#include <eigen_conversions/eigen_msg.h>

#include <moveit/ompl_interface/planning_context_manager.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_request.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>

#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h>
#include <moveit/ompl_interface/parameterization/joint_space/constrained_planning_state_space.h>

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

  void testSimpleRequest(const std::vector<double>& start, const std::vector<double>& goal)
  {
    // create all the test specific input necessary to make the getPlanningContext call possible
    planning_interface::PlannerConfigurationSettings pconfig_settings;
    pconfig_settings.group = group_name_;
    pconfig_settings.name = group_name_;
    pconfig_settings.config = { { "enforce_joint_model_state_space", "0" },
                                { "use_ompl_constrained_state_space", "0" } };

    planning_interface::PlannerConfigurationMap pconfig_map{ { pconfig_settings.name, pconfig_settings } };
    moveit_msgs::MoveItErrorCodes error_code;
    planning_interface::MotionPlanRequest request = createRequest(start, goal);

    // setup the planning context manager
    ompl_interface::PlanningContextManager pcm(robot_model_, constraint_sampler_manager_);
    pcm.setPlannerConfigurations(pconfig_map);

    // see if it returns the expected planning context
    auto pc = pcm.getPlanningContext(planning_scene_, request, error_code, node_handle_, false);

    EXPECT_NE(pc->getOMPLSimpleSetup(), nullptr);
    auto ss = dynamic_cast<ompl_interface::JointModelStateSpace*>(pc->getOMPLStateSpace().get());
    EXPECT_NE(ss, nullptr);

    planning_interface::MotionPlanDetailedResponse res;
    bool success = pc->solve(res);
    EXPECT_TRUE(success);
  }

  void testPathConstraints(const std::vector<double>& start, const std::vector<double>& goal)
  {
    // create all the test specific input necessary to make the getPlanningContext call possible
    planning_interface::PlannerConfigurationSettings pconfig_settings;
    pconfig_settings.group = group_name_;
    pconfig_settings.name = group_name_;
    pconfig_settings.config = { { "enforce_joint_model_state_space", "0" },
                                { "use_ompl_constrained_state_space", "0" } };

    planning_interface::PlannerConfigurationMap pconfig_map{ { pconfig_settings.name, pconfig_settings } };
    moveit_msgs::MoveItErrorCodes error_code;
    planning_interface::MotionPlanRequest request = createRequest(start, goal);

    // give it some more time, as rejection sampling of the path constraints occasionally takes some time
    request.allowed_planning_time = 10.0;

    // create path constraints around start state,  to make sure they are satisfied
    robot_state_->setJointGroupPositions(joint_model_group_, start);
    Eigen::Isometry3d ee_pose = robot_state_->getGlobalLinkTransform(ee_link_name_);
    // geometry_msgs::Quaternion ee_orientation;
    // tf::quaternionEigenToMsg(Eigen::Quaterniond(ee_pose.rotation()), ee_orientation);
    // request.path_constraints.orientation_constraints.push_back(createOrientationConstraint(ee_orientation));

    request.path_constraints.position_constraints.push_back(createPositionConstraint(
        { ee_pose.translation().x(), ee_pose.translation().y(), ee_pose.translation().z() }, { 1.0, 1.0, 1.0 }));

    // setup the planning context manager
    ompl_interface::PlanningContextManager pcm(robot_model_, constraint_sampler_manager_);
    pcm.setPlannerConfigurations(pconfig_map);

    // see if it returns the expected planning context
    auto pc = pcm.getPlanningContext(planning_scene_, request, error_code, node_handle_, false);

    EXPECT_NE(pc->getOMPLSimpleSetup(), nullptr);

    // As the joint_model_group_ has not IK solver initialized, we still get a joint model state space here,
    // so not really a good test. TODO(jeroendm)
    auto ss = dynamic_cast<ompl_interface::JointModelStateSpace*>(pc->getOMPLStateSpace().get());
    EXPECT_NE(ss, nullptr);

    planning_interface::MotionPlanDetailedResponse res;
    bool success = pc->solve(res);
    EXPECT_TRUE(success);
  }

  void testOMPLConstrainedPlanning(const std::vector<double>& start, const std::vector<double>& goal)
  {
    // create all the test specific input necessary to make the getPlanningContext call possible
    planning_interface::PlannerConfigurationSettings pconfig_settings;
    pconfig_settings.group = group_name_;
    pconfig_settings.name = group_name_;
    pconfig_settings.config = { { "enforce_joint_model_state_space", "0" },
                                { "enforce_constrained_state_space", "1" },
                                { "projection_evaluator", "joints(joint_1,joint_2)" },
                                { "longest_valid_segment_fraction", "0.05" } };  //,
    // { "default_planner_config", "RRTConnect" } };
    // RRTConnect configuration
    planning_interface::PlannerConfigurationSettings rrt_config;
    rrt_config.group = group_name_;
    rrt_config.name = group_name_ + "[RRTConnect]";
    rrt_config.config = { { "enforce_joint_model_state_space", "0" },
                          { "enforce_constrained_state_space", "1" },
                          { "projection_evaluator", "joints(joint_1,joint_2)" },
                          { "longest_valid_segment_fraction", "0.05" },
                          { "type", "geometric::RRTConnect" },
                          { "range", "0.0" } };

    planning_interface::PlannerConfigurationMap pconfig_map{ { pconfig_settings.name, pconfig_settings },
                                                             { rrt_config.name, rrt_config } };
    moveit_msgs::MoveItErrorCodes error_code;
    planning_interface::MotionPlanRequest request = createRequest(start, goal);

    // create path constraints around start state, to make sure they are satisfied
    robot_state_->setJointGroupPositions(joint_model_group_, start);
    Eigen::Isometry3d ee_pose = robot_state_->getGlobalLinkTransform(ee_link_name_);
    // geometry_msgs::Quaternion ee_orientation;
    // tf::quaternionEigenToMsg(Eigen::Quaterniond(ee_pose.rotation()), ee_orientation);
    // request.path_constraints.orientation_constraints.push_back(createOrientationConstraint(ee_orientation));
    request.path_constraints.position_constraints.push_back(createPositionConstraint(
        { ee_pose.translation().x(), ee_pose.translation().y(), ee_pose.translation().z() }, { 1.0, 1.0, 1.0 }));

    request.planner_id = "RRTConnect";
    // give it some more time
    request.allowed_planning_time = 20.0;

    // setup the planning context manager
    ompl_interface::PlanningContextManager pcm(robot_model_, constraint_sampler_manager_);
    pcm.setPlannerConfigurations(pconfig_map);

    // Check if it returns the expected planning context
    auto pc = pcm.getPlanningContext(planning_scene_, request, error_code, node_handle_, false);
    EXPECT_NE(pc->getOMPLSimpleSetup(), nullptr);

    auto ss = dynamic_cast<ompl_interface::ConstrainedPlanningStateSpace*>(pc->getOMPLStateSpace().get());
    ASSERT_NE(ss, nullptr);

    EXPECT_EQ(ss->getDimension(), num_dofs_);

    planning_interface::MotionPlanDetailedResponse res;
    bool success = pc->solve(res);
    EXPECT_TRUE(success);
  }

  // /***************************************************************************
  //  * END Test implementation
  //  * ************************************************************************/

protected:
  void SetUp() override
  {
    // create all the fixed input necessary for all planning context managers
    constraint_sampler_manager_ = std::make_shared<constraint_samplers::ConstraintSamplerManager>();
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
  }

  void TearDown() override
  {
  }

  /** Create a planning request to plan from a given start state to a joint space goal. **/
  planning_interface::MotionPlanRequest createRequest(const std::vector<double>& start,
                                                      const std::vector<double>& goal) const
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

  /** \brief Helper function to create a position constraint. **/
  moveit_msgs::PositionConstraint createPositionConstraint(std::array<double, 3> position,
                                                           std::array<double, 3> dimensions)
  {
    shape_msgs::SolidPrimitive box;
    box.type = shape_msgs::SolidPrimitive::BOX;
    box.dimensions.resize(3);
    box.dimensions[shape_msgs::SolidPrimitive::BOX_X] = dimensions[0];
    box.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = dimensions[1];
    box.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = dimensions[2];

    geometry_msgs::Pose box_pose;
    box_pose.position.x = position[0];
    box_pose.position.y = position[1];
    box_pose.position.z = position[2];
    box_pose.orientation.w = 1.0;

    moveit_msgs::PositionConstraint position_constraint;
    position_constraint.header.frame_id = base_link_name_;
    position_constraint.link_name = ee_link_name_;
    position_constraint.constraint_region.primitives.push_back(box);
    position_constraint.constraint_region.primitive_poses.push_back(box_pose);

    return position_constraint;
  }

  ompl_interface::ModelBasedStateSpacePtr state_space_;
  ompl_interface::ModelBasedPlanningContextSpecification planning_context_spec_;
  ompl_interface::ModelBasedPlanningContextPtr planning_context_;
  planning_scene::PlanningScenePtr planning_scene_;

  constraint_samplers::ConstraintSamplerManagerPtr constraint_sampler_manager_;

  /** Ideally we add an IK plugin to the joint_model_group_ to test the PoseModel state space, using the pluginlib to
   * load the default KDL plugin? **/
  // std::shared_ptr<kinematics::KinematicsBase> ik_plugin_;

  ros::NodeHandle node_handle_;
};

/***************************************************************************
 * Run all tests on the Panda robot
 * ************************************************************************/
// class PandaTestPlanningContext : public TestPlanningContext
// {
// protected:
//   PandaTestPlanningContext() : TestPlanningContext("panda", "panda_arm")
//   {
//   }
// };

// TEST_F(PandaTestPlanningContext, testSimpleRequest)
// {
//   // use the panda "ready" state from the srdf config as start state
//   // we know this state should be within limits and self-collision free
//   testSimpleRequest({ 0, -0.785, 0, -2.356, 0, 1.571, 0.785 }, { 0, -0.785, 0, -2.356, 0, 1.571, 0.685 });
// }

// TEST_F(PandaTestPlanningContext, testPathConstraints)
// {
//   testPathConstraints({ 0, -0.785, 0, -2.356, 0, 1.571, 0.785 }, { 0, -0.785, 0, -2.356, 0, 1.571, 0.685 });
// }

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

// TEST_F(FanucTestPlanningContext, testSimpleRequest)
// {
//   testSimpleRequest({ 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0.1 });
// }

// TEST_F(FanucTestPlanningContext, testPathConstraints)
// {
//   testPathConstraints({ 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0.1 });
// }

TEST_F(FanucTestPlanningContext, testOMPLConstrainedPlanning)
{
  testOMPLConstrainedPlanning({ 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0.1 });
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