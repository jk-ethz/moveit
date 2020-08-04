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
#include <moveit/ompl_interface/planning_context_manager.h>
#include <moveit/ompl_interface/parameterization/joint_space/joint_model_state_space.h>
#include <moveit/ompl_interface/parameterization/work_space/pose_model_state_space.h>

#include <gtest/gtest.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>

/** \brief Loads a robot model and executes tests on it.
 *
 * All tests are implemented in a generic test fixture, so it is
 * easy to run them on different robots.
 *
 * based on
 * https://stackoverflow.com/questions/38207346/specify-constructor-arguments-for-a-google-test-fixture/38218657
 * (answer by PiotrNycz)
 *
 * (jeroendm) Ideally I want to get rid of the ros node handle in the test
 * so it is independent of the rostest framework.
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
    // I need a ROS node handle to create the PlanningContextManager
    nh_ = ros::NodeHandle("/planning_context_manager_test");

    // load robot
    robot_model_ = moveit::core::loadTestingRobotModel(robot_name_);

    // alternative way to load robot model, which requires the robot description
    // to be loaded on the parameter server, but could have an IK solver initialized.
    // robot_model_loader::RobotModelLoaderPtr robot_model_loader(
    //     new robot_model_loader::RobotModelLoader("robot_description"));
    // robot_model_ = robot_model_loader->getModel();

    robot_state_ = std::make_shared<robot_state::RobotState>(robot_model_);
    joint_model_group_ = robot_state_->getJointModelGroup(group_name_);
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);

    // extract useful parameters for tests
    num_dofs_ = joint_model_group_->getVariableCount();
    ee_link_name_ = joint_model_group_->getLinkModelNames().back();
    base_link_name_ = robot_model_->getRootLinkName();
  };

  void TearDown() override
  {
  }

  /** \brief Create a joint position vector with values 0.1, 0.2, 0.3, ... where the length depends on the number of
   * joints in the robot. * */
  moveit::core::RobotState getDeterministicState() const
  {
    Eigen::VectorXd joint_positions(num_dofs_);
    double value = 0.1;
    for (std::size_t i = 0; i < num_dofs_; ++i)
    {
      joint_positions[i] = value;
      value += 0.1;
    }
    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();
    state.copyJointGroupPositions(joint_model_group_, joint_positions);
    return state;
  }

  /** \brief Create a simple and valid MotionPlanRequest as input for getPlanningContext. **/
  planning_interface::MotionPlanRequest createExampleRequest() const
  {
    planning_interface::MotionPlanRequest request;

    // general settings
    request.group_name = group_name_;
    request.allowed_planning_time = 5.0;

    // start state
    moveit::core::RobotState start_state = getDeterministicState();
    moveit::core::robotStateToRobotStateMsg(start_state, request.start_state);

    // goal state = slightly modified version of start state
    moveit::core::RobotState goal_state(start_state);
    goal_state.setVariablePosition(0, goal_state.getVariablePosition(0) + 0.2);
    request.goal_constraints.push_back(
        kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group_, 0.001));

    return request;
  }

  /** \brief Add orientation constraints to a motion planning request. **/
  void addPathConstraints(planning_interface::MotionPlanRequest& request) const
  {
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = ee_link_name_;
    ocm.header.frame_id = base_link_name_;
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    request.path_constraints.orientation_constraints.push_back(ocm);
  }

  void testStateSpaceSelection()
  {
    planning_interface::MotionPlanRequest request = createExampleRequest();

    // create a planner configurations map with settings
    // here we can add "enforce_joint_model_state_space" in future tests
    planning_interface::PlannerConfigurationSettings pconfig;
    pconfig.group = group_name_;
    pconfig.name = group_name_;
    planning_interface::PlannerConfigurationMap pconfig_map;
    pconfig_map[pconfig.name] = pconfig;

    // setup a valid ConstraintSamplerManager that we can test
    auto constraint_sampler_manager = std::make_shared<constraint_samplers::ConstraintSamplerManager>();
    ompl_interface::PlanningContextManager planning_context_manager(robot_model_, constraint_sampler_manager);
    planning_context_manager.setPlannerConfigurations(pconfig_map);

    // the default planning request without path constraints should use a JointModelStateSpace
    moveit_msgs::MoveItErrorCodes error_code;
    auto pc = planning_context_manager.getPlanningContext(planning_scene_, request, error_code, nh_, false);
    EXPECT_EQ(pc->getOMPLStateSpace()->getParameterizationType(),
              ompl_interface::JointModelStateSpace::PARAMETERIZATION_TYPE);

    // adding path constraints (and having an ik solver, and no visibility or joint constraints)
    // should result in planning with a PoseModelStateSpace
    // THIS TEST IS FAILING because there is no ik solver found
    addPathConstraints(request);
    pc = planning_context_manager.getPlanningContext(planning_scene_, request, error_code, nh_, false);
    EXPECT_EQ(pc->getOMPLStateSpace()->getParameterizationType(),
              ompl_interface::PoseModelStateSpace::PARAMETERIZATION_TYPE);

    // adding joint constraints should result in JointModelStateSpace again
    // TODO(jeroendm)

    // TODO(jeroendm) add cases for visibility constraints and enforce_joint_model_state_space

    EXPECT_TRUE(true);
  }

protected:
  const std::string group_name_;
  const std::string robot_name_;

  ros::NodeHandle nh_;
  moveit::core::RobotModelPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  const robot_state::JointModelGroup* joint_model_group_;
  planning_scene::PlanningScenePtr planning_scene_;

  std::size_t num_dofs_;
  std::string base_link_name_;
  std::string ee_link_name_;
};

/***************************************************************************
 * Run all tests on the Panda robot
 * ************************************************************************/
class PandaTests : public LoadTestRobotBaseClass
{
protected:
  PandaTests() : LoadTestRobotBaseClass("panda", "panda_arm")
  {
  }
};

TEST_F(PandaTests, testStateSpaceSelection)
{
  testStateSpaceSelection();
}

/***************************************************************************
 * Run all tests on the Fanuc robot
 * ************************************************************************/
class FanucTests : public LoadTestRobotBaseClass
{
protected:
  FanucTests() : LoadTestRobotBaseClass("fanuc", "manipulator")
  {
  }
};

TEST_F(FanucTests, testStateSpaceSelection)
{
  testStateSpaceSelection();
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