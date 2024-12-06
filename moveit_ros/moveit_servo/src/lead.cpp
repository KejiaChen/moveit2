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

#include <chrono>
using namespace std::literals;

namespace
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.lead");
constexpr size_t LOG_THROTTLE_PERIOD = 10;  // sec
constexpr size_t MAX_QUEUE_SIZE = 5;   

// Helper template for declaring and getting ros param
template <typename T>
void declareOrGetParam(T& output_value, const std::string& param_name, const rclcpp::Node::SharedPtr& node,
                       const rclcpp::Logger& logger, const T default_value = T{})
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
    RCLCPP_WARN_STREAM(logger, "InvalidParameterTypeException(" << param_name << "): " << e.what());
    RCLCPP_ERROR_STREAM(logger, "Error getting parameter \'" << param_name << "\', check parameter type in YAML file");
    throw e;
  }

  RCLCPP_INFO_STREAM(logger, "Found parameter - " << param_name << ": " << output_value);
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
{
  readROSParams();

  robot_model_ = planning_scene_monitor_->getRobotModel();

  // Use the C++ interface that Servo provides
  servo_ = std::make_unique<moveit_servo::Servo>(node_, servo_parameters_, planning_scene_monitor_);
  servo_->start();

  // Connect to MTC ROS interfaces
  target_traj_sub_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "target_joint_trajectory", rclcpp::SystemDefaultsQoS(),
      [this](const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr& msg) { return targetJointTrajectoryCallback(msg); });

  // Publish outgoing joint commands to the Servo object
  joint_command_pub_ = node_->create_publisher<control_msgs::msg::JointJog>(
      servo_->getParameters()->joint_command_in_topic, rclcpp::SystemDefaultsQoS());

  // Subscribe to velocity scale
   velocity_scale_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
        "/velocity_scale", 10, std::bind(&Lead::velocityScaleCallback, this, std::placeholders::_1));
}

// Process a queue of trajectories
LeadStatusCode Lead::processTrajectory(){
  while (rclcpp::ok() && !stop_requested_)
  {
    trajectory_msgs::msg::JointTrajectory trajectory;

    {
      // Lock the queue and fetch the next trajectory if available
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (!trajectory_queue_.empty())
      {
        trajectory = trajectory_queue_.front();
        trajectory_queue_.pop();
      }
    }

    // If no trajectory is available, wait and retry
    if (trajectory.points.empty())
    {
      RCLCPP_INFO(LOGGER, "No trajectories in queue. Waiting for input...");
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    // Track the entire trajectory
    executeTrajectory(trajectory);

    // Perform post-motion reset
    doPostMotionReset();
  }
}

// Execution of a single trajectory
LeadStatusCode Lead::executeTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory)
{
  if (trajectory.points.empty())
  {
    RCLCPP_WARN(LOGGER, "Received empty trajectory. Nothing to execute.");
    return LeadStatusCode::EMPTY_TRAJECTORY;
  }

  double velocity_scale = 1.0; // Initial velocity scaling factor

  for (size_t i = 0; i < trajectory.points.size() - 1 && !stop_requested_; ++i)
{
    const auto& current_point = trajectory.points[i];
    const auto& next_point = trajectory.points[i + 1];
    executeTrajectorySegment(current_point, next_point, trajectory.joint_names);
}

if (stop_requested_)
{
    RCLCPP_INFO(LOGGER, "Stop requested during trajectory tracking. Halting.");
    doPostMotionReset();
    return LeadStatusCode::STOP_REQUESTED;
}

RCLCPP_INFO(LOGGER, "Trajectory execution complete.");
return LeadStatusCode::SUCCESS;
}


// Execution from current point to next point
void Lead::executeTrajectorySegment(const trajectory_msgs::msg::JointTrajectoryPoint& current_point,
                              const trajectory_msgs::msg::JointTrajectoryPoint& next_point,
                              const std::vector<std::string>& joint_names)
{
  rclcpp::Time segment_start_time = node_->now();

  while (rclcpp::ok() && !stop_requested_)
{
    // Compute the time difference for this segment
    double time_diff = (rclcpp::Duration(next_point.time_from_start) - rclcpp::Duration(current_point.time_from_start)).seconds();
    if (time_diff <= 0)
    {
        RCLCPP_WARN(LOGGER, "Invalid time difference. Skipping segment.");
        break;
    }

    // Fetch the latest velocity scale
    double velocity_scale = getVelocityScale();
    double scaled_time_diff = time_diff / velocity_scale;

    // Compute joint velocities
    std::vector<double> velocities;
    for (size_t j = 0; j < current_point.positions.size(); ++j)
    {
        double velocity = (next_point.positions[j] - current_point.positions[j]) / scaled_time_diff;
        velocities.push_back(velocity);
    }

    // Publish JointJog message
    publishJointJog(joint_names, velocities);

    // Sleep based on Servo's publish period
    if (!loop_rate_.sleep())
    {
        RCLCPP_WARN_STREAM_THROTTLE(LOGGER, *node_->get_clock(), LOG_THROTTLE_PERIOD, "Control loop missed.");
    }

    // Check if the segment duration is complete
    if ((node_->now() - segment_start_time).seconds() >= scaled_time_diff)
    {
        break;
    }
}

}

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

void Lead::publishJointJog(const std::vector<std::string>& joint_names, const std::vector<double>& velocities)
{
  control_msgs::msg::JointJog jog_msg;
  jog_msg.header.stamp = node_->now();
  jog_msg.joint_names = joint_names;
  jog_msg.velocities = velocities;
  jog_msg.duration = 0.0; // Zero duration implies immediate execution by MoveIt Servo

  joint_command_pub_->publish(jog_msg);
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

  // x_pid_config_.dt = publish_period;
  // y_pid_config_.dt = publish_period;
  // z_pid_config_.dt = publish_period;
  // angular_pid_config_.dt = publish_period;

  double windup_limit;
  declareOrGetParam(windup_limit, ns + ".windup_limit", node_, LOGGER);
  // x_pid_config_.windup_limit = windup_limit;
  // y_pid_config_.windup_limit = windup_limit;
  // z_pid_config_.windup_limit = windup_limit;
  // angular_pid_config_.windup_limit = windup_limit;

  // declareOrGetParam(x_pid_config_.k_p, ns + ".x_proportional_gain", node_, LOGGER);
  // declareOrGetParam(x_pid_config_.k_p, ns + ".x_proportional_gain", node_, LOGGER);
  // declareOrGetParam(y_pid_config_.k_p, ns + ".y_proportional_gain", node_, LOGGER);
  // declareOrGetParam(z_pid_config_.k_p, ns + ".z_proportional_gain", node_, LOGGER);
  // declareOrGetParam(x_pid_config_.k_i, ns + ".x_integral_gain", node_, LOGGER);
  // declareOrGetParam(y_pid_config_.k_i, ns + ".y_integral_gain", node_, LOGGER);
  // declareOrGetParam(z_pid_config_.k_i, ns + ".z_integral_gain", node_, LOGGER);
  // declareOrGetParam(x_pid_config_.k_d, ns + ".x_derivative_gain", node_, LOGGER);
  // declareOrGetParam(y_pid_config_.k_d, ns + ".y_derivative_gain", node_, LOGGER);
  // declareOrGetParam(z_pid_config_.k_d, ns + ".z_derivative_gain", node_, LOGGER);

  // declareOrGetParam(angular_pid_config_.k_p, ns + ".angular_proportional_gain", node_, LOGGER);
  // declareOrGetParam(angular_pid_config_.k_i, ns + ".angular_integral_gain", node_, LOGGER);
  // declareOrGetParam(angular_pid_config_.k_d, ns + ".angular_derivative_gain", node_, LOGGER);
}

void Lead::targetJointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (trajectory_queue_.size() >= MAX_QUEUE_SIZE)
    {
        RCLCPP_WARN(LOGGER, "Trajectory queue is full. Dropping oldest trajectory.");
        trajectory_queue_.pop();  // Remove oldest trajectory
    }
    trajectory_queue_.push(*msg);
    RCLCPP_INFO(LOGGER, "Received new trajectory with %zu waypoints.", msg->points.size());
}


void Lead::stopMotion()
{
  stop_requested_ = true;

  // Send a 0 command to Servo to halt arm motion
  auto msg = moveit::util::make_shared_from_pool<control_msgs::msg::JointJog>();
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    msg->joint_names = trajectory_queue_.back().joint_names;
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
