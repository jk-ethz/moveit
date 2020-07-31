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

#pragma once

#include <ompl/base/Constraint.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/Constraints.h>

namespace ompl_interface
{
/** \brief Abstract base class for differen types of constraints, implementations of ompl::base::Constraint
 *
 * To create a constrained state space in OMPL, we need a model of the constraints.
 * Any generic model that can be written in the form of equality constraints F(joint_positions) = 0
 * will work. In the remainder of this file, derived classes implement position and orientation constraints.
 * */
class OMPLConstraint : public ompl::base::Constraint
{
public:
  OMPLConstraint() = delete;
  OMPLConstraint(moveit::core::RobotModelConstPtr robot_model, const std::string& group, const unsigned int num_dofs,
                 const unsigned int num_constraints_ = 3);

  /** \brief Initialize the constraint based on the content of the MoveIt constraint message, found in the
  *  path_constraints field of the motion planning request. **/
  virtual bool initialize(const moveit_msgs::Constraints& constraints);

  /** OMPL's main constraint evaluation function. This must be overwritten to implement a constraint and represents the
   * constraint as function(joint_positions) = 0.
   * */
  virtual void function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const;

  /** \brief Optionally the jacobian of the above constraint function can be implemented, which can speed up the
   * calculations. **/
  virtual void jacobian(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::MatrixXd> out) const;

protected:
  /** \brief Wrapper for forward kinematics calculated by MoveIt's Robot State.
   *
   * I'm counting on compiler optimization to reduce the cost of returing Isometry3d by value.
   * */
  Eigen::Isometry3d forwardKinematics(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;

  /** \brief MoveIt's robot representation for kinematic calculations. **/
  robot_model::RobotModelConstPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  const robot_state::JointModelGroup* joint_model_group_;

  /** \brief Robot link the constraints are applied to. */
  std::string link_name_;

  /** \brief target for equality constraints, nominal value for inequality constraints. */
  Eigen::Vector3d target_position_;

  /** \brief target for equality constraints, nominal value for inequality constraints. */
  Eigen::Quaterniond target_orientation_;

  /** \brief indicate which position or orientation dimensions are constrainted.
   *
   * For example, for position constraints on the x and z position, it would be [0, 2].
   * For orientation constraints on the first two rotation angles it would be TODO(jeroendm) [0, 1] or [3, 4].
   * */
  std::vector<std::size_t> constrained_dimensions_;

public:
  // Macro for classes containing fixed size eigen vectors that are dynamically allocated when used.
  // https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class PositionEqualityConstraint : public OMPLConstraint
{
public:
  PositionEqualityConstraint() = delete;
  PositionEqualityConstraint(moveit::core::RobotModelConstPtr robot_model, const std::string& group,
                             const unsigned int num_dofs);

  virtual bool initialize(const moveit_msgs::Constraints& constraints) override;
  virtual void function(const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::Ref<Eigen::VectorXd> out) const override;
};

bool containsValidPositionConstraint(const moveit_msgs::Constraints& constraints);
}