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

#pragma once

#include <ostream>

#include <ompl/base/Constraint.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/Constraints.h>
#include <moveit/macros/class_forward.h>

namespace ompl_interface
{
MOVEIT_CLASS_FORWARD(BaseConstraint);
MOVEIT_CLASS_FORWARD(PositionConstraint);

/** \brief Represents upper and lower bound on a scalar value (double).
 *
 * This is necessary because an OMPL ConstrainedStateSpace requires a model of the constriants given as a generic
 * equalities
 *  f1(joint_values) = 0
 *  f2(joint_values) = 0
 *  f3(joint_values) = 0
 *  ...
 *
 * So we use a penalty function to convert bounds to an equality constraint.
 * If you do need equality constriants, you can represent them by setting the upper bound and lower bound almost equal.
 *
 * **/
struct Bounds
{
  double lower, upper;

  /** \brief Distance to region inside bounds
   *
   * Distance of a given value outside the bounds, zero inside the bounds.
   * Creates a penalty function that looks like this:
   *
   * (penalty) ^
   *           | \         /
   *           |  \       /
   *           |   \_____/
   *           |----------------> (variable to be constrained)
   *
   * TODO(jeroendm) Change it to a penalty function that has a continuous derivative, so we can use AtlasStateSpace and
   * TangentBundleStateSpace.
   * */
  double penalty(double value) const;

  /** \brief Derivative of the penalty function
   * ^
   * |
   * | -1-1-1 0 0 0 +1+1+1
   * |------------------------>
   * **/
  double derivative(double value) const;
};

/** \brief Pretty printing of bounds. **/
std::ostream& operator<<(std::ostream& os, const ompl_interface::Bounds& bound);

/** \brief Abstract base class for differen types of constraints, implementations of ompl::base::Constraint
 *
 * To create a constrained state space in OMPL, we need a model of the constraints, that can be written in the form of
 * equality constraints F(joint_values) = 0. This class uses `ompl_interface::Bounds` defined above, to convert:
 *
 *    lower_bound < scalar value < upper bound
 *
 * into an equation of the form f(x) = 0.
 *
 * The 'scalar value' can be error on the position or orientation of a link relative to a desired reference position, or
 * any other error metric that can be calculated using the `moveit::core::RobotModel` and
 * `moveit::core::JointModelGroup`.
 * */
class BaseConstraint : public ompl::base::Constraint
{
public:
  BaseConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group, const unsigned int num_dofs,
                 const unsigned int num_cons_ = 3);

  /** \brief initialize constraint based on message content.
   *
   * This is necessary because we cannot call the pure virtual
   * parseConstraintsMsg method from the constructor of this class.
   * */
  void init(const moveit_msgs::Constraints& constraints);

  /** OMPL's main constraint evaluation function.
   *
   *  OMPL requires you to override at least "function" which represents the constraint F(q) = 0
   * */
  virtual void function(const Eigen::Ref<const Eigen::VectorXd>& joint_values, Eigen::Ref<Eigen::VectorXd> out) const;

  /** \brief Jacobian of the constraint function.
   *
   * Optionally you can also provide dF(q)/dq, the Jacobian of  the constriants.
   *
   * */
  virtual void jacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values, Eigen::Ref<Eigen::MatrixXd> out) const;

  /** \brief Wrapper for forward kinematics calculated by MoveIt's Robot State.
   *
   * TODO(jeroendm) Are these actually const, as the robot state is modified? How come it works?
   * Also, output arguments could be significantly more performant,
   * but MoveIt's robot state does not support passing Eigen::Ref objects at the moment.
   * */
  Eigen::Isometry3d forwardKinematics(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;

  /** \brief Calculate the robot's geometric Jacobian using MoveIt's Robot State.
   *
   * Ideally I would pass the output agrument from OMPL's jacobian function directly,
   * but I cannot pass an object of type , Eigen::Ref<Eigen::MatrixXd> to MoveIt's
   * Jacobian method.
   * */
  Eigen::MatrixXd robotGeometricJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;

  /** \brief Parse bounds on position and orientation parameters from MoveIt's constraint message.
   *
   * This can be non-trivial given the often complex structure of these messages.
   * For the current example with equality constraints it could be to simple
   * to have this separate function instead of using the init function directly.
   * */
  virtual void parseConstraintMsg(const moveit_msgs::Constraints& constraints) = 0;

  /** \brief For inequality constraints: calculate the value of the parameter that is being constraint by the bounds.
   *
   * In this Position constraints case, it calculates the x, y and z position
   * of the end-effector. This error is then converted in generic equality constraints in the implementation of
   * `ompl_interface::BaseConstraint::function`.
   *
   * This method can be bypassed if you want to override `ompl_interface::BaseConstraint::function directly and ignore
   * the bounds calculation.
   * */
  virtual Eigen::VectorXd calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const
  {
    ROS_ERROR_STREAM("Constraint method calcError was not overridded, so it should not be used.");
    return Eigen::VectorXd::Zero(getCoDimension());
  }

  /** \brief For inequality constraints: calculate the Jacobian for the current parameters that are being constraints.
   *   *
   * This error jacobian, as the name suggests, is only the jacobian of the position / orientation / ... error.
   * It does not take into acount the derivative of the penalty functions defined in the Bounds class.
   * This correction is added in the implementation of of BaseConstraint::jacobian.
   *
   * This method can be bypassed if you want to override `ompl_interface::BaseConstraint::jacobian directly and ignore
   * the bounds calculation.
   *
   * TODO(jeroendm), Maybe also use an output agrument as in `ompl::base::Constraint::jacobian(x, out)` for better
   * performance?
   * */
  virtual Eigen::MatrixXd calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const
  {
    ROS_ERROR_STREAM("Constraint method calcErrorJacobian was not overridded, so it should not be used.");
    return Eigen::MatrixXd::Zero(getCoDimension(), n_);
  }

  /** \brief Return the link name to which the constraint is applied (mostly for debugging). **/
  const std::string& getLinkName()
  {
    return link_name_;
  }

  const Eigen::Vector3d getTargetPosition()
  {
    return target_position_;
  }

  const Eigen::Quaterniond getTargetOrientation()
  {
    return target_orientation_;
  }

protected:
  // MoveIt's robot representation for kinematic calculations
  robot_model::RobotModelConstPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  const robot_state::JointModelGroup* joint_model_group_;

  /** \brief Robot link the constraints are applied to. */
  std::string link_name_;

  /** \brief Upper and lower bounds on constrained variables. */
  std::vector<Bounds> bounds_;

  /** \brief target for equality constraints, nominal value for inequality constraints. */
  Eigen::Vector3d target_position_;

  /** \brief target for equality constraints, nominal value for inequality constraints. */
  Eigen::Quaterniond target_orientation_;

public:
  // Macro for classes containing fixed size eigen vectors that are dynamically allocated when used.
  // https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** \brief Box shaped position constraints
 *
 * Reads bounds on x, y and z position from a position constraint
 * at constraint_region.primitives[0].dimensions.
 * Where the primitive has to be of type `shape_msgs/SolidPrimitive.BOX`.
 *
 * These bounds are applied around the nominal position and orientation
 * of the box.
 *
 * */
class PositionConstraint : public BaseConstraint
{
public:
  PositionConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group, const unsigned int num_dofs)
    : BaseConstraint(robot_model, group, num_dofs)
  {
  }
  virtual void parseConstraintMsg(const moveit_msgs::Constraints& constraints) override;
  virtual Eigen::VectorXd calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const override;
  virtual Eigen::MatrixXd calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const override;
};

MOVEIT_CLASS_FORWARD(OrientationConstraint);
/** \brief Orientation constraints parameterized using exponential coordinates.
 *
 * An orientation constraints is modelled as a deviation from a target orientation.
 * The deviation is represented using exponential coordinates. A three element vector represents the rotation axis
 * multiplied with the angle in radians around this axis.
 *
 *  R_error = R_end_effector ^ (-1) * R_target
 *  R_error -> rotation angle and axis           (using Eigen3)
 *  error = angle * axis                         (vector with three elements)
 *
 *  And then the constraints can be written as
 *
 *     - absolute_x_axis_tolerance / 2 < error[0] < absolute_x_axis_tolerance / 2
 *     - absolute_y_axis_tolerance / 2 < error[1] < absolute_y_axis_tolerance / 2
 *     - absolute_z_axis_tolerance / 2 < error[2] < absolute_z_axis_tolerance / 2
 *
 * **IMPORTANT** It is NOT how orientation error is handled in the default MoveIt constraint samplers, where XYZ
 * intrinsic euler angles are used. Using exponential coordinates is analog to how orientation Error is calculated in
 * the TrajOpt  motion planner.
 *
 * */
class OrientationConstraint : public BaseConstraint
{
public:
  OrientationConstraint(robot_model::RobotModelConstPtr robot_model, const std::string& group,
                        const unsigned int num_dofs)
    : BaseConstraint(robot_model, group, num_dofs)
  {
  }

  void jacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values, Eigen::Ref<Eigen::MatrixXd> out) const override
  {
    ompl::base::Constraint::jacobian(joint_values, out);
  }

  void parseConstraintMsg(const moveit_msgs::Constraints& constraints) override;
  Eigen::VectorXd calcError(const Eigen::Ref<const Eigen::VectorXd>& x) const override;
  // Eigen::MatrixXd calcErrorJacobian(const Eigen::Ref<const Eigen::VectorXd>& x) const override;
};

/** \brief Conversion matrix to go from angular velocity in the world frame to
 * angle axis equivalent.
 *
 * Based on:
 * https://ethz.ch/content/dam/ethz/special-interest/mavt/robotics-n-intelligent-systems/rsl-dam/documents/RobotDynamics2016/RD2016script.pdf
 *
 * */
Eigen::Matrix3d angularVelocityToAngleAxis(double angle, const Eigen::Vector3d& axis);

/** \brief Extract position constraints from the MoveIt message.
 *
 * Assumes there is a single primitive of type `shape_msgs/SolidPrimitive.BOX`.
 * The dimensions of the box are the bounds on the deviation of the link origin from
 * the target pose, given in constraint_regions.primitive_poses[0].
 * */
std::vector<Bounds> positionConstraintMsgToBoundVector(const moveit_msgs::PositionConstraint& pos_con);

/** \brief Extract orientation constraints from the MoveIt message
 *
 * These bounds are assumed to be centered around the target orientation / desired orientation
 * given in the "orientation" field in the message.
 * These bounds represent orientation error between the desired orientation and the current orientation of the
 * end-effector.
 *
 * The "absolute_x_axis_tolerance", "absolute_y_axis_tolerance" and "absolute_z_axis_tolerance" are interpreted as
 * the width of the tolerance regions around the target orientation, represented using exponential coordinates.
 *
 * */
std::vector<Bounds> orientationConstraintMsgToBoundVector(const moveit_msgs::OrientationConstraint& ori_con);

/** \brief Factory to create constraints based on what is in the MoveIt constraint message. **/
std::shared_ptr<BaseConstraint> createOMPLConstraint(robot_model::RobotModelConstPtr robot_model,
                                                     const std::string& group,
                                                     const moveit_msgs::Constraints& constraints);
}  // namespace ompl_interface