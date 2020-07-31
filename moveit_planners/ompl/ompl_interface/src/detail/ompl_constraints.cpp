/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Jeroen De Maeyer
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

#include <moveit/ompl_interface/detail/ompl_constraints.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/Constraints.h>
#include <eigen_conversions/eigen_msg.h>

namespace ompl_interface
{
constexpr char LOGNAME[] = "ompl_constraint";

/****************************
 * Base class for constraints
 * **************************/
OMPLConstraint::OMPLConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                               const unsigned int num_dofs, const unsigned int num_constraints_)
  : robot_model_(std::move(robot_model)), ompl::base::Constraint(num_dofs, num_constraints_)
{
  // Setup Moveit's robot model for kinematic calculations
  robot_state_.reset(new robot_state::RobotState(robot_model_));
  robot_state_->setToDefaultValues();
  joint_model_group_ = robot_state_->getJointModelGroup(group);
}

Eigen::Isometry3d OMPLConstraint::forwardKinematics(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  robot_state_->setJointGroupPositions(joint_model_group_, joint_values);
  return robot_state_->getGlobalLinkTransform(link_name_);
}

/******************************************
 * Position constraints
 * ****************************************/
PositionEqualityConstraint::PositionEqualityConstraint(moveit::core::RobotModelConstPtr robot_model,
                                                       const std::string& group, const unsigned int num_dofs)
  : OMPLConstraint(robot_model, group, num_dofs)
{
}

bool PositionEqualityConstraint::initialize(const moveit_msgs::Constraints& constraints)
{
  if (!containsValidPositionConstraint(constraints))
  {
    return false;
  }

  moveit_msgs::PositionConstraint position_constraint = constraints.position_constraints.at(0);
  shape_msgs::SolidPrimitive box = position_constraint.constraint_region.primitives.at(0);

  // extract target position and orientation
  geometry_msgs::Point target_point = position_constraint.constraint_region.primitive_poses.at(0).position;
  target_position_ << target_point.x, target_point.y, target_point.z;

  tf::quaternionMsgToEigen(position_constraint.constraint_region.primitive_poses.at(0).orientation,
                           target_orientation_);

  // parse what dimensions should be constrained
  constrained_dimensions_.clear();
  for (std::size_t dim{ 0 }; dim < 3; ++dim)
  {
    if (box.dimensions[dim] == 0.0)
    {
      constrained_dimensions_.push_back(dim);
    }
  }
  return true;
}

void PositionEqualityConstraint::function(const Eigen::Ref<const Eigen::VectorXd>& x,
                                          Eigen::Ref<Eigen::VectorXd> out) const
{
  auto position_error =
      target_orientation_.matrix().transpose() * (forwardKinematics(x).translation() - target_position_);
  for (std::size_t dim{ 0 }; dim < getCoDimension(); ++dim)
  {
    // TODO(jeroendm) The number of constraints, size of codimensions, must be changed at construction time, which is
    // not the case yet...
    out[dim] = position_error[constrained_dimensions_[dim]];
  }
}

bool containsValidPositionConstraint(const moveit_msgs::Constraints& constraints)
{
  if (constraints.position_constraints.size() == 0)
  {
    ROS_ERROR_STREAM_NAMED(
        LOGNAME, "No position constraints found in path constraints. Cannot initialize PositionEqualityConstraints.");
    return false;
  }

  moveit_msgs::PositionConstraint position_constraint = constraints.position_constraints.at(0);
  if (position_constraint.constraint_region.primitives.size() == 0)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "No primitive region found in position constraints. (Meshed are not supported.)");
    return false;
  }

  shape_msgs::SolidPrimitive box = position_constraint.constraint_region.primitives.at(0);
  if (box.type != box.BOX)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Only the BOX primitive is supported for position constraints.");
    return false;
  }

  return true;
}
}