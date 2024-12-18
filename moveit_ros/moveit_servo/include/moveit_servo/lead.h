/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
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
 *   * Neither the name of PickNik Inc. nor the names of its
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
/*
   Author: Andy Zelenak
   Desc: Servoing. Track a pose setpoint in real time.
*/

#pragma once

#include <atomic>
#include <control_toolbox/pid.hpp>
#include <moveit_servo/make_shared_from_pool.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo.h>
#include <optional>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>
#include <std_msgs/msg/bool.hpp>

// Conventions:
// Calculations are done in the planning_frame_ unless otherwise noted.

namespace moveit_servo
{
struct PIDConfig
{
  double dt = 0.001;       // Time step
  double k_p = 100;        // Proportional gain
  double k_i = 0;          // Integral gain
  double k_d = 0.1;        // Derivative gain
  double windup_limit = 0.1; // Anti-windup limit
};

enum class LeadStatusCode : int8_t
{
  INVALID = -1,
  SUCCESS = 0,
  EMPTY_TRAJECTORY = 1,
  NO_RECENT_TARGET_JOINT = 2,
  STOP_REQUESTED = 3
};

const std::unordered_map<LeadStatusCode, std::string> LEAD_STATUS_CODE_MAP = {
  { LeadStatusCode::INVALID, "Invalid" },
  { LeadStatusCode::SUCCESS, "Success" },
  { LeadStatusCode::EMPTY_TRAJECTORY, "Empty joint trajectories" },
  { LeadStatusCode::NO_RECENT_TARGET_JOINT, "No recent target joint position" },
  { LeadStatusCode::STOP_REQUESTED, "Stop requested" }
};

/**
 * Class Lead - Tracks and executes joint trajectories using MoveIt Servo.
 * Adjust velocity scaling factor online.
 */
class Lead
{
public:
  /** Constructor */
  Lead(const rclcpp::Node::SharedPtr& node, const ServoParameters::SharedConstPtr& servo_parameters,
       const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);

  /** Set the joint names */
  void setJointNames(const std::vector<std::string>& joint_names);

  /** Execute a segment of the trajectory */
  LeadStatusCode moveToJoint(const double target_joint_timeout);

  // /**Return true if a target joint has been received within timeout */
  // bool Lead::haveRecentTargetJoint(const double timespan);

  // /**Return true if a joint position has been received within timeout */
  // bool Lead::haveRecentJointPosition(const double timespan);

  /** Publish JointJog commands for servoing */
  void publishJointJog(const std::vector<double>& velocities);

  /** \brief Change PID parameters. Motion is stopped before the update */
  void updatePIDConfig(const double proportional_gain, const double integral_gain, const double derivative_gain);

  // void getPIDErrors(double& x_error, double& y_error, double& z_error, double& orientation_error);

  /** Stop motion and reset flags */
  void stopMotion();

  /** Servo instance (public for direct access to Servo functions like `setPaused`) */
  std::unique_ptr<moveit_servo::Servo> servo_;

private:
  /** Initialize ROS parameters */
  void readROSParams();

  /** \brief Initialize a PID controller and add it to vector of controllers */
  void initializePID(const PIDConfig& pid_config, std::vector<control_toolbox::Pid>& pid_vector);

  /** Check if the joint values are within tolerance of the target joint */
  bool satisfiesJointTolerance(const std::vector<double>& joint_values, const double tolerance);

  /** Get current joint values */
  std::vector<double> getCurrentJointValues(const std::string& group_name);

  /** Callback for target joint trajectories */
  void targetJointCallback(const trajectory_msgs::msg::JointTrajectoryPoint::ConstSharedPtr& msg);

  /** \brief Use PID controllers to calculate a full spatial velocity toward a joint position */
  std::vector<double> calculateJointCommand(const std::vector<double>& current_joint_values);

  /** Perform cleanup and reset PID after motion */
  void doPostMotionReset();

  /** Callback for velocity scaling factor */
  void velocityScaleCallback(const std_msgs::msg::Float64::SharedPtr msg);

  /** Thread-safe access to velocity scale */
  double getVelocityScale();

  rclcpp::Node::SharedPtr node_;
  ServoParameters::SharedConstPtr servo_parameters_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  moveit::core::RobotModelConstPtr robot_model_;
  std::vector<std::string> joint_names_;

  // Joint control
  std::string move_group_name_;
  rclcpp::WallRate loop_rate_;

  // ROS publishers and subscribers
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_command_pub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr target_joint_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_scale_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr waypoint_reached_pub_;

  // PID controllers
  std::vector<control_toolbox::Pid> joint_pids_;
  // Joint PID configs
  PIDConfig joint_pid_config_;
  double joint_reach_tolerance_;

  // Joint Waypoints
  bool target_joint_available_;
  trajectory_msgs::msg::JointTrajectoryPoint target_joint_;
  mutable std::mutex target_joint_mtx_;

  // Time
  rclcpp::Time robot_joint_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

  // Velocity scale management
  double velocity_scale_ = 1.0;
  std::mutex scale_mutex_;

  // Stop flag
  std::atomic<bool> stop_requested_;

  // Angular error for tolerance checks
  std::optional<double> angular_error_;
};

// Shared pointer alias
using LeadPtr = std::shared_ptr<Lead>;

}  // namespace moveit_servo
