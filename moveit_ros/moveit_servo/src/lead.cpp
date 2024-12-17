/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the leading conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the leading disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the leading
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

#include <moveit_servo/lead.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit/robot_state/robot_state.h>

#include <chrono>
using namespace std::literals;

namespace
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.lead");
constexpr size_t LOG_THROTTLE_PERIOD = 10;  // sec
constexpr size_t MAX_QUEUE_SIZE = 10;   

// Helper template for declaring and getting ros param
template <typename T>
void declareOrGetParam(T& output_value, const std::string& param_name, const rclcpp::Node::SharedPtr& node,
                       const rclcpp::Logger& LOGGER, const T default_value = T{})
{
  try
  {
    if (node->has_parameter(param_name))
    {
      node->get_parameter<T>(param_name, output_value);
    }
    else
    {
      output_value = node->declare_parameter<T>(param_name, default_value);
    }
  }
  catch (const rclcpp::exceptions::InvalidParameterTypeException& e)
  {
    RCLCPP_WARN_STREAM(LOGGER, "InvalidParameterTypeException(" << param_name << "): " << e.what());
    RCLCPP_ERROR_STREAM(LOGGER, "Error getting parameter \'" << param_name << "\', check parameter type in YAML file");
    throw e;
  }

  RCLCPP_INFO_STREAM(LOGGER, "Found parameter - " << param_name << ": " << output_value);
}
}  // namespace

namespace moveit_servo
{
Lead::Lead(const rclcpp::Node::SharedPtr& node, const ServoParameters::SharedConstPtr& servo_parameters,
                           const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
  : node_(node)
  , servo_parameters_(servo_parameters)
  , planning_scene_monitor_(planning_scene_monitor)
  , loop_rate_(1.0 / servo_parameters->publish_period)
  , velocity_scale_(1.0)
  , stop_requested_(false)
  , target_joint_available_(false)
{
  readROSParams();

  robot_model_ = planning_scene_monitor_->getRobotModel();

  // Use the C++ interface that Servo provides
  servo_ = std::make_unique<moveit_servo::Servo>(node_, servo_parameters_, planning_scene_monitor_);
  servo_->start();

  target_joint_sub_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectoryPoint>(
      "target_joint", rclcpp::SystemDefaultsQoS(),
      [this](const trajectory_msgs::msg::JointTrajectoryPoint::ConstSharedPtr& msg) { return targetJointCallback(msg); });

  // Publish outgoing joint commands to the Servo object
  joint_command_pub_ = node_->create_publisher<control_msgs::msg::JointJog>(
      servo_->getParameters()->joint_command_in_topic, rclcpp::SystemDefaultsQoS());

  // Publish the signal of reaching the waypoint
  waypoint_reached_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "waypoint_reached", rclcpp::SystemDefaultsQoS());

  // Subscribe to velocity scale
  //  velocity_scale_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
  //       "/velocity_scale", 10, std::bind(&Lead::velocityScaleCallback, this, std::placeholders::_1));
}

void Lead::setJointNames(const std::vector<std::string>& joint_names)
{
  joint_names_ = joint_names;
  RCLCPP_INFO_STREAM(LOGGER, "Setting joint names: ");
  for (const auto& name : joint_names_)
  {
    RCLCPP_INFO_STREAM(LOGGER, name << " ");
  }
}

// Execution from current point to next point
LeadStatusCode Lead::moveToJoint(const double target_joint_timeout)
{
  // Reset stop requested flag before starting motions
  stop_requested_ = false;
  bool target_joint_available = false;
  const double control_period = servo_parameters_->publish_period; // Control loop period

  rclcpp::Time start_time = node_->now();

  // while ((!haveRecentTargetPose(target_joint_timeout) || !haveRecentEndEffectorPose(target_joint_timeout)) &&
  //        ((node_->now() - start_time).seconds() < target_joint_timeout))
  // {
  //   robot_joint_stamp_ = node_->now();
  //   std::this_thread::sleep_for(1ms);
  // }

  // Wait for a target joint to be available
  while (rclcpp::ok() && !target_joint_available)
  {
    {
      std::lock_guard<std::mutex> lock(target_joint_mtx_);
      target_joint_available = target_joint_available_;
    }

    // if (!target_joint_available && (node_->now() - start_time).seconds() >= target_joint_timeout)
    // {
    //   RCLCPP_ERROR_STREAM(LOGGER, "No recent target joint data. Aborting.");
    //   return LeadStatusCode::NO_RECENT_TARGET_JOINT;
    // }

    std::this_thread::sleep_for(1ms);  // Small sleep to avoid busy waiting
  }

  // Reset start time for execution
  start_time = node_->now();

  bool waypoint_reached = false;
  std::vector<double> previous_velocities(target_joint_.positions.size(), 0.0);
  while (rclcpp::ok())
  {
    // Get the current joint values
    auto current_joint = getCurrentJointValues(move_group_name_);

    // check if this waypoint is reached
    if (satisfiesJointTolerance(current_joint, 0.01))
    {
      if (!waypoint_reached)
      {
        RCLCPP_INFO_STREAM(LOGGER, "The target joint is achieved!");
        waypoint_reached = true;
      }
      std_msgs::msg::Bool msg;
      msg.data = true;
      waypoint_reached_pub_->publish(msg); // TODO@Kejia: change to service
      continue;
    }

    // Fetch the latest velocity scale
    // double velocity_scale = getVelocityScale();
    double velocity_scale = 1.0;
    
    // Compute joint velocities TODO@Kejia: use current joint values
    std::vector<double> velocities;
    for (size_t j = 0; j < target_joint_.velocities.size(); ++j)
    {
        // double velocity = target_joint_.velocities[j] * velocity_scale;

        // double velocity = (target_joint_.positions[j] - current_joint[j]) * velocity_scale;
        // velocities.push_back(velocity);

        double position_error = target_joint_.positions[j] - current_joint[j];
        double target_velocity = (position_error / control_period) * velocity_scale;
        velocities.push_back(target_velocity);

    }

    if (stop_requested_)
    {
      RCLCPP_INFO_STREAM(LOGGER, "Halting servo motion, a stop was requested.");
      doPostMotionReset();
      return LeadStatusCode::STOP_REQUESTED;
    }

    // Publish JointJog message
    publishJointJog(velocities);

    if (!loop_rate_.sleep())
    {
      RCLCPP_WARN_STREAM_THROTTLE(LOGGER, *node_->get_clock(), LOG_THROTTLE_PERIOD, "Target control rate was missed");
    }

  }

  doPostMotionReset();
  return LeadStatusCode::SUCCESS;
}

// bool Lead::haveRecentTargetJoint(const double timespan)
// {
//   std::lock_guard<std::mutex> lock(target_joint_mtx_);
//   return ((node_->now() - target_joint_.header.stamp).seconds() < timespan);
// }

// bool Lead::haveRecentJointPosition(const double timespan)
// {
//   return ((node_->now() - command_frame_transform_stamp_).seconds() < timespan);
// }

void Lead::velocityScaleCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(scale_mutex_);
  velocity_scale_ = msg->data;
  RCLCPP_INFO(LOGGER, "Updated velocity scale: %.2f", velocity_scale_);
}

double Lead::getVelocityScale()
{
  std::lock_guard<std::mutex> lock(scale_mutex_);
  return velocity_scale_;
}

void Lead::publishJointJog(const std::vector<double>& velocities)
{
  control_msgs::msg::JointJog jog_msg;
  jog_msg.header.stamp = node_->now(); // This is important!
  jog_msg.joint_names = joint_names_;
  // jog_msg.displacements = displacements;
  jog_msg.velocities = velocities;
  jog_msg.duration = 0.0; // Zero duration implies immediate execution by MoveIt Servo

  RCLCPP_INFO_STREAM(LOGGER, "Publishing joint velocities: " << jog_msg.velocities[0] << ", " << jog_msg.velocities[1] << ", " << jog_msg.velocities[2]
                      << ", " << jog_msg.velocities[3] << ", " << jog_msg.velocities[4] << ", " << jog_msg.velocities[5]);

  // RCLCPP_INFO_STREAM(LOGGER, "Publishing joint displacements: " << jog_msg.displacements[0] << ", " << jog_msg.displacements[1] << ", " << jog_msg.displacements[2]
  //                     << ", " << jog_msg.displacements[3] << ", " << jog_msg.displacements[4] << ", " << jog_msg.displacements[5]);

  joint_command_pub_->publish(std::move(jog_msg));
}

void Lead::readROSParams()
{
  const std::string ns = "moveit_servo";

  // declareOrGetParam(planning_frame_, ns + ".planning_frame", node_, LOGGER);
  declareOrGetParam(move_group_name_, ns + ".move_group_name", node_, LOGGER);

  if (!planning_scene_monitor_->getRobotModel()->hasJointModelGroup(move_group_name_))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Unable to find the specified joint model group: " << move_group_name_);
  }

  double publish_period;
  declareOrGetParam(publish_period, ns + ".publish_period", node_, LOGGER);

  double windup_limit;
  declareOrGetParam(windup_limit, ns + ".windup_limit", node_, LOGGER);
}

std::vector<double> Lead::getCurrentJointValues(const std::string& group_name)
{
    // Ensure the PlanningSceneMonitor is initialized and running
    if (!planning_scene_monitor_ || !planning_scene_monitor_->getStateMonitor())
    {
        RCLCPP_ERROR(LOGGER, "PlanningSceneMonitor or StateMonitor is not initialized.");
        return {};
    }
    // Get the current robot state
    auto current_state = planning_scene_monitor_->getStateMonitor()->getCurrentState();
    if (!current_state)
    {
        RCLCPP_ERROR(LOGGER, "Failed to retrieve the current robot state.");
        return {};
    }

    // Get the joint model group for the specified group name
    const moveit::core::JointModelGroup* joint_model_group = current_state->getJointModelGroup(group_name);
    if (!joint_model_group)
    {
        RCLCPP_ERROR(LOGGER, "Joint group '%s' not found.", group_name.c_str());
        return {};
    }

    // Retrieve the current joint values
    std::vector<double> joint_values;
    current_state->copyJointGroupPositions(joint_model_group, joint_values);
    return joint_values;
}


bool Lead::satisfiesJointTolerance(const std::vector<double>& joint_values, const double tolerance)
{
  std::lock_guard<std::mutex> lock(target_joint_mtx_);
  if (!target_joint_available_)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "No target joint available to compare to");
    return false;
  }

  if (joint_values.size() != target_joint_.positions.size())
  {
    RCLCPP_WARN_STREAM(LOGGER, "Joint values size does not match target joint size");
    return false;
  }

  for (size_t i = 0; i < joint_values.size(); ++i)
  {
    if (std::abs(joint_values[i] - target_joint_.positions[i]) > tolerance)
    {
      return false;
    }
  }

  return true;
}

void Lead::targetJointCallback(const trajectory_msgs::msg::JointTrajectoryPoint::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lock(target_joint_mtx_);
    target_joint_ = *msg;
    target_joint_available_ = true;
    RCLCPP_INFO_STREAM(LOGGER, "Received target joint position");
}


void Lead::stopMotion()
{
  stop_requested_ = true;

  // Send a 0 command to Servo to halt arm motion
  auto msg = moveit::util::make_shared_from_pool<control_msgs::msg::JointJog>();
  {
    std::lock_guard<std::mutex> lock(target_joint_mtx_);
    msg->joint_names = joint_names_;
  }
  msg->header.stamp = node_->now();
  joint_command_pub_->publish(*msg);
}

void Lead::doPostMotionReset()
{
  stopMotion();
  stop_requested_ = false;
  angular_error_ = {};

}
}  // namespace moveit_servo
